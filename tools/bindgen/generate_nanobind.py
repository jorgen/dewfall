#!/usr/bin/env python3
"""Stage 2 of the dewfall binding generator: dew_api.json -> nanobind C++.

Consumes the generator-agnostic semantic layer and emits one C++ file that
builds the flat `dew` extension module. Python-specific policy lives here:
  - `py.skip` annotations drop declarations from the Python surface only
  - handle classes become holder structs with GIL-releasing destructors
  - multi-out-param getters return dicts; bool-gated getters return None
  - callback registrars become kwargs of Python callables with generated
    GIL-acquiring trampolines
  - a custom-binding registry routes the file-convert callback machinery to
    hand-written code (bindings/python/custom/file_convert_callbacks.h)

The emitter refuses to guess: any construct it cannot map cleanly is a hard
error naming the declaration, so API growth is either bound or explicitly
annotated -- never silently dropped.
"""

import argparse
import json
import sys

# C++/windows-macro identifiers that cannot be used as parameter names.
_CPP_RESERVED = {
    "near",
    "far",
    "new",
    "delete",
    "default",
    "register",
    "template",
    "typename",
    "operator",
    "this",
    "class",
    "struct",
    "union",
    "export",
    "min",
    "max",
}

_CUSTOM_CLASS_SNIPPETS = {
    # C function name -> (bound class, snippet emitted inside that class's register fn)
    "dew_converter_set_file_converter_callbacks": (
        "Converter",
        "dewpy::bind_set_file_converter_callbacks(cls, m);",
    ),
    "dew_laszip_callbacks": ("Converter", "dewpy::bind_use_laszip_callbacks(cls);"),
    # add_data_for_attribute is py.skip (a bare const void* is not a safe Python parameter); the shim
    # binds a numpy-aware version. It takes over add_attribute too, to record the declared type that
    # the value-array size is then checked against.
    "dew_converter_add_attribute": ("Converter", "dewpy::bind_amend(cls);"),
    # The C request lifecycle (opaque handle, borrowed buffers, explicit release) is not what a
    # Python caller wants; Dataset.query_box() runs it end to end and hands back NumPy arrays.
    "dew_dataset_request_region": ("Dataset", "dewpy::bind_query_box(cls);\n  dewpy::bind_query_submit<PyRequest>(cls);"),
    # The pump is how a Python event loop drives the library without blocking: a wake callback says
    # "look again", poll() dispatches. Both are py.skip in the header because their C signatures carry
    # a raw user_ptr the generators cannot express.
    "dew_pump_create": ("Pump", "dewpy::bind_pump_async(cls);"),
    # Hangs the request accessors off the GENERATED Request holder. It must be the generated one:
    # the opaque dew_request_t already produces an (empty) Request class, registered last, which
    # would overwrite a second nb::class_ of the same name.
    "dew_request_status": ("Request", "dewpy::bind_request_async(cls);"),
}


class EmitError(Exception):
    pass


def _san(name):
    return name + "_" if name in _CPP_RESERVED else name


def _py_skipped(entity):
    return entity.get("annotations", {}).get("py.skip") is True


def _doc_literal(doc):
    if not doc:
        return None
    return f'R"doc({doc})doc"'


class Emitter:
    def __init__(self, document):
        self.raw = document["raw"]
        self.semantic = document["semantic"]
        self.headers = [h for module in document["modules"] for h in module["headers"]]

        self.raw_functions = {f["name"]: f for f in self.raw["functions"]}
        self.raw_structs = {s["name"]: s for s in self.raw["structs"]}
        self.callbacks = {c["name"]: c for c in self.raw["callbacks"]}
        self.classes = {c["handle_type"]: c for c in self.semantic["classes"]}
        self.value_structs = {s["c_name"]: s for s in self.semantic["value_structs"]}

        self.holders = {}  # handle type -> holder struct name (only non-py.skip classes)
        for cls in self.semantic["classes"]:
            if not _py_skipped(cls):
                self.holders[cls["handle_type"]] = "Py" + cls["bound_name"]

        # value-struct emission strategy: plain | token | wrapper | skip
        self.struct_kind = {}
        for struct in self.semantic["value_structs"]:
            self.struct_kind[struct["c_name"]] = self._classify_struct(struct)

        self.covered = set()  # C function names accounted for
        self.skipped = []  # (function, reason)
        self.blocks = []  # namespace-scope definitions (ctx structs, trampolines)
        self.registers = []  # register_* function bodies
        self.register_calls = []

    # ------------------------------------------------------------------
    # struct classification
    # ------------------------------------------------------------------

    def _classify_struct(self, struct):
        if _py_skipped(struct) or struct["is_callback_struct"]:
            return "skip"
        has_callback = any(f["type"]["kind"] == "callback" for f in struct["fields"])
        if has_callback:
            return "token"
        for field in struct["fields"]:
            t = field["type"]
            if t["kind"] == "pointer":
                pointee = t["pointee"]
                if pointee["kind"] == "void" and field.get("count_field"):
                    continue  # buffer field, handled by wrapper
                raise EmitError(
                    f"struct {struct['c_name']}.{field['c_name']}: pointer field needs an "
                    f"//= arrays: annotation or //= py.skip on the struct"
                )
        if any(f["type"]["kind"] == "pointer" for f in struct["fields"]):
            return "wrapper"
        return "plain"

    def _struct_cpp_type(self, c_name):
        """The C++ type a bound struct is exposed as (wrapper or the C struct)."""
        kind = self.struct_kind.get(c_name)
        if kind == "wrapper":
            return "Py" + self.value_structs[c_name]["bound_name"]
        return c_name

    # ------------------------------------------------------------------
    # enums / constants / value structs
    # ------------------------------------------------------------------

    def emit_enums(self):
        lines = ["static void register_enums(nb::module_ &m)", "{"]
        for enum in self.semantic["enums"]:
            if _py_skipped(enum):
                continue
            doc = _doc_literal(enum["doc"])
            doc_arg = f", {doc}" if doc else ""
            lines.append(f"  // {enum['c_name']} ({enum['header']})")
            lines.append(f"  nb::enum_<{enum['c_name']}>(m, \"{enum['bound_name']}\", nb::is_arithmetic(){doc_arg})")
            for value in enum["values"]:
                lines.append(f"    .value(\"{value['bound_name']}\", {value['c_name']})")
            lines[-1] += ";"
        lines += ["}", ""]
        self.registers.append("\n".join(lines))
        self.register_calls.append("register_enums(m);")

    def emit_constants(self):
        lines = ["static void register_constants(nb::module_ &m)", "{"]
        for constant in self.semantic["constants"]:
            value = constant["value"].replace("\\", "\\\\").replace('"', '\\"')
            lines.append(f"  m.attr(\"{constant['bound_name']}\") = \"{value}\";")
        lines += ["}", ""]
        self.registers.append("\n".join(lines))
        self.register_calls.append("register_constants(m);")

    def _array_field_props(self, struct_c, field):
        """Property lambdas for a fixed-array struct field."""
        name = field["c_name"]
        t = field["type"]
        element = t["element"]
        total = 1
        for extent in t["extents"]:
            total *= extent
        if element["kind"] == "builtin" and element["name"] == "char" and len(t["extents"]) == 1:
            return [
                f'    .def_prop_ro("{name}", [](const {struct_c} &s) {{',
                f"      return nb::str(s.{name}, strnlen(s.{name}, {total}));",
                "    })",
            ]
        if element["kind"] == "record":
            count = field.get("count_field")
            if not count:
                raise EmitError(f"struct {struct_c}.{name}: record array needs //= arrays: with a count field")
            element_cpp = self._struct_cpp_type(element["name"])
            if self.struct_kind.get(element["name"]) not in ("plain",):
                raise EmitError(f"struct {struct_c}.{name}: element {element['name']} is not a plain bound struct")
            return [
                f'    .def_prop_ro("{name}", [](const {struct_c} &s) {{',
                "      nb::list out;",
                f"      uint32_t n = s.{count} < {total} ? static_cast<uint32_t>(s.{count}) : {total}u;",
                f"      for (uint32_t i = 0; i < n; ++i) out.append(nb::cast(s.{name}[i]));",
                "      return out;",
                "    })",
            ]
        if element["kind"] not in ("builtin", "enum"):
            raise EmitError(f"struct {struct_c}.{name}: unsupported array element kind {element['kind']}")
        elem_name = element["name"]
        return [
            f'    .def_prop_rw("{name}",',
            f"      [](const {struct_c} &s) {{",
            f"        std::array<{elem_name}, {total}> v{{}};",
            f"        std::memcpy(v.data(), &s.{name}[0], sizeof(v));",
            "        return v;",
            "      },",
            f"      []({struct_c} &s, const std::array<{elem_name}, {total}> &v) {{",
            f"        std::memcpy(&s.{name}[0], v.data(), sizeof(v));",
            "      })",
        ]

    def emit_value_structs(self):
        lines = ["static void register_value_structs(nb::module_ &m)", "{"]
        for struct in self.semantic["value_structs"]:
            c_name = struct["c_name"]
            kind = self.struct_kind[c_name]
            if kind == "skip":
                continue
            bound = struct["bound_name"]
            doc = _doc_literal(struct["doc"])
            doc_arg = f", {doc}" if doc else ""
            lines.append(f"  // {c_name} ({struct['header']}) [{kind}]")
            if kind == "token":
                lines.append(f"  nb::class_<{c_name}>(m, \"{bound}\"{doc_arg});")
                continue
            if kind == "wrapper":
                wrapper = "Py" + bound
                lines.append(f"  nb::class_<{wrapper}>(m, \"{bound}\"{doc_arg})")
                lines.append("    .def(nb::init<>())")
                for field in struct["fields"]:
                    t = field["type"]
                    name = field["c_name"]
                    if t["kind"] == "pointer":
                        count = field["count_field"]
                        lines += [
                            f'    .def_prop_rw("{name}",',
                            f"      [](const {wrapper} &s) -> nb::object {{",
                            f"        return s.keep_{name}.is_valid() ? nb::object(s.keep_{name}) : nb::none();",
                            "      },",
                            f"      []({wrapper} &s, nb::bytes b) {{",
                            f"        s.keep_{name} = b;",
                            f"        s.c.{name} = const_cast<char *>(b.c_str());",
                            f"        s.c.{count} = static_cast<decltype(s.c.{count})>(b.size());",
                            "      })",
                        ]
                    elif t["kind"] in ("builtin", "enum"):
                        # count fields stay readable; harmless
                        lines.append(f'    .def_prop_ro("{name}", [](const {wrapper} &s) {{ return s.c.{name}; }})')
                    else:
                        raise EmitError(f"struct {c_name}.{name}: unsupported field in wrapper struct")
                lines[-1] += ";"
                # wrapper holder definition goes to namespace scope
                keeps = [f["c_name"] for f in struct["fields"] if f["type"]["kind"] == "pointer"]
                holder = [f"struct {wrapper}", "{", f"  {c_name} c{{}};"]
                holder += [f"  nb::object keep_{k};" for k in keeps]
                holder += ["};", ""]
                self.blocks.append("\n".join(holder))
                continue
            # plain
            lines.append(f"  nb::class_<{c_name}>(m, \"{bound}\"{doc_arg})")
            lines.append("    .def(nb::init<>())")
            for field in struct["fields"]:
                t = field["type"]
                name = field["c_name"]
                field_doc = _doc_literal(field["doc"])
                if t["kind"] == "array":
                    lines += self._array_field_props(c_name, field)
                elif t["kind"] in ("builtin", "enum"):
                    doc_suffix = f", {field_doc}" if field_doc else ""
                    lines.append(f"    .def_rw(\"{name}\", &{c_name}::{name}{doc_suffix})")
                elif t["kind"] == "record":
                    if self.struct_kind.get(t["name"]) != "plain":
                        raise EmitError(f"struct {c_name}.{name}: nested struct {t['name']} is not plain-bound")
                    lines.append(f"    .def_rw(\"{name}\", &{c_name}::{name})")
                else:
                    raise EmitError(f"struct {c_name}.{name}: unsupported field kind {t['kind']}")
            lines[-1] += ";"
        lines += ["}", ""]
        self.registers.append("\n".join(lines))
        self.register_calls.append("register_value_structs(m);")

    # ------------------------------------------------------------------
    # holder structs
    # ------------------------------------------------------------------

    def emit_holders(self):
        for cls in self.semantic["classes"]:
            if _py_skipped(cls):
                continue
            holder = self.holders[cls["handle_type"]]
            handle = cls["handle_type"]
            lines = [f"struct {holder}", "{", f"  {handle} *h = nullptr;"]
            lines.append("  std::vector<std::unique_ptr<dewpy::cb_ctx_base>> ctxs;")
            lines.append(f"  {holder}() = default;")
            lines.append(f"  {holder}(const {holder} &) = delete;")
            lines.append(f"  {holder} &operator=(const {holder} &) = delete;")
            lines.append(f"  {holder}({holder} &&other) noexcept : h(other.h), ctxs(std::move(other.ctxs)) {{ other.h = nullptr; }}")
            if cls["destructor"]:
                destroy = cls["destructor"]["function"]
                drain = cls["destructor"].get("annotations", {}).get("py.drain_on_destroy")
                lines += [
                    f"  ~{holder}()",
                    "  {",
                    "    if (h)",
                    "    {",
                    "      // destroy joins the library's internal threads; callbacks may need the GIL",
                    "      nb::gil_scoped_release release;",
                ]
                if isinstance(drain, str):
                    lines += [
                        "      // Python destroys implicitly (del, rebinding, GC, an exception",
                        "      // unwinding), so the pipeline must be quiesced first: tearing it",
                        "      // down mid-flight aborts the process inside the thread pool.",
                        f"      {drain}(h);",
                    ]
                lines += [
                    f"      {destroy}(h);",
                    "    }",
                    "    // ctxs (Python callables) die only after destroy returned",
                    "  }",
                ]
            else:
                lines.append(f"  ~{holder}() = default;  // non-owning view handle")
            lines += ["};", ""]
            self.blocks.append("\n".join(lines))

    # ------------------------------------------------------------------
    # method emission
    # ------------------------------------------------------------------

    def _is_str_buffer_record(self, name):
        struct = self.raw_structs.get(name)
        if not struct or len(struct["fields"]) != 2:
            return False
        data, size = struct["fields"]
        t = data["type"]
        return (
            t["kind"] == "pointer"
            and t["pointee"]["kind"] == "builtin"
            and t["pointee"]["name"] == "char"
            and size["type"]["kind"] == "builtin"
        )

    def _emit_plain_method(self, cls, method, mode):
        """mode: 'method' | 'init' | 'static_ctor' | 'free'"""
        fn = self.raw_functions[method["function"]]
        n_params = len(fn["params"])
        call_args = [None] * n_params
        params = []  # (cpp type, cpp name, py name, default)
        pre, post = [], []
        keep_alive_args = []  # 1-based positions among visible python args
        results = method["results"]
        holder = self.holders.get(cls["handle_type"]) if cls else None

        if method["self_param"] is not None:
            call_args[method["self_param"]] = "self.h"

        if method["error_convention"] == "error_out_owned":
            pre.append("dew_error_t *error_ = nullptr;")
            call_args[method["error_param"]] = "&error_"
        elif method["error_convention"] == "error_arg_caller_owned":
            pre.append("dewpy::scoped_error error_;")
            call_args[method["error_param"]] = "error_.get()"

        out_string = next((r for r in results if r["role"] == "out_string"), None)

        for arg in method["args"]:
            role = arg["role"]
            name = _san(arg["bound_name"])
            py_name = arg["bound_name"]
            t = arg["type"]
            i = arg["c_params"][0]
            if role in ("callback_struct", "callback_fn"):
                raise EmitError(f"{method['function']}: callback args must go through _emit_callback_method")
            if role == "scalar":
                params.append((t["name"], name, py_name, None))
                call_args[i] = name
            elif role == "enum":
                params.append((t["name"], name, py_name, None))
                call_args[i] = name
            elif role == "string_in":
                params.append(("std::string_view", name, py_name, None))
                call_args[i] = f"{name}.data()"
                call_args[arg["c_params"][1]] = f"static_cast<{arg['len_type']['name']}>({name}.size())"
            elif role == "array_in":
                if len(t["extents"]) != 1:
                    raise EmitError(f"{method['function']}: multi-dimensional array param unsupported")
                params.append((f"std::array<{t['element']['name']}, {t['extents'][0]}>", name, py_name, None))
                call_args[i] = f"{name}.data()"
            elif role == "struct_in":
                if arg.get("by_value"):
                    record = t["name"]
                    kind = self.struct_kind.get(record)
                    if kind == "wrapper":
                        params.append((f"const {self._struct_cpp_type(record)} &", name, py_name, None))
                        call_args[i] = f"{name}.c"
                    elif kind in ("plain", "token"):
                        params.append((f"const {record} &", name, py_name, None))
                        call_args[i] = name
                        if kind == "token":
                            keep_alive_args.append(len(params))
                    else:
                        raise EmitError(f"{method['function']}: struct arg {record} is not bound for Python")
                else:
                    record = t["pointee"]["name"]
                    if self.struct_kind.get(record) != "plain":
                        raise EmitError(f"{method['function']}: struct pointer arg {record} is not plain-bound")
                    params.append((f"const {record} &", name, py_name, None))
                    call_args[i] = f"const_cast<{record} *>(&{name})" if not t["pointee"]["const"] else f"&{name}"
            elif role == "handle_in":
                other = self.holders.get(arg["handle"])
                if not other:
                    raise EmitError(f"{method['function']}: handle arg {arg['handle']} has no Python class")
                params.append((f"{other} &", name, py_name, None))
                call_args[i] = f"{name}.h"
                keep_alive_args.append(len(params))
            elif role == "buffer_array_in":
                element = arg["element_type"]
                count_i = arg["c_params"][1]
                count_cast = arg["count_type"]["name"]
                if element["kind"] == "record" and self._is_str_buffer_record(element["name"]):
                    record = element["name"]
                    params.append(("const std::vector<std::string> &", name, py_name, None))
                    pre += [
                        f"std::vector<{record}> {name}_c({name}.size());",
                        f"for (size_t i_ = 0; i_ < {name}.size(); ++i_)",
                        "{",
                        f"  {name}_c[i_].data = {name}[i_].data();",
                        f"  {name}_c[i_].size = static_cast<uint32_t>({name}[i_].size());",
                        "}",
                    ]
                    call_args[i] = f"{name}_c.data()"
                    call_args[count_i] = f"static_cast<{count_cast}>({name}_c.size())"
                elif element["kind"] == "record" and self.struct_kind.get(element["name"]) == "plain":
                    record = element["name"]
                    params.append((f"std::vector<{record}> &", name, py_name, None))
                    call_args[i] = f"{name}.data()"
                    call_args[count_i] = f"static_cast<{count_cast}>({name}.size())"
                else:
                    raise EmitError(f"{method['function']}: unsupported buffer array element")
            else:
                raise EmitError(f"{method['function']}: unhandled arg role {role}")

        # results -> locals
        result_exprs = []  # (name, expr)
        gated = False
        return_needed = None
        for result in results:
            role = result["role"]
            t = result["type"]
            if result["source"] == "return":
                if len(results) > 1 and t.get("name") == "bool":
                    gated = True
                    return_needed = "bool"
                else:
                    return_needed = "value"
                continue
            name = _san(result["bound_name"]) + "_out"
            if role == "out_param":
                if result.get("borrowed"):
                    raise EmitError(f"{method['function']}: borrowed pointer out-param needs custom binding")
                scalar = t["pointee"]["name"]
                pre.append(f"{scalar} {name}{{}};")
                call_args[result["c_params"][0]] = f"&{name}"
                result_exprs.append((result["bound_name"], name))
            elif role == "array_out":
                element = t["element"]["name"]
                extent = t["extents"][0]
                pre.append(f"std::array<{element}, {extent}> {name}{{}};")
                call_args[result["c_params"][0]] = f"{name}.data()"
                result_exprs.append((result["bound_name"], name))
            elif role == "struct_out":
                record = t["pointee"]["name"]
                if self.struct_kind.get(record) != "plain":
                    raise EmitError(f"{method['function']}: struct out {record} is not plain-bound")
                pre.append(f"{record} {name}{{}};")
                call_args[result["c_params"][0]] = f"&{name}"
                result_exprs.append((result["bound_name"], name))
            elif role == "out_string":
                pass  # whole call is wrapped below
            else:
                raise EmitError(f"{method['function']}: unhandled result role {role}")

        missing = [fn["params"][idx]["name"] for idx, value in enumerate(call_args) if value is None]
        if out_string:
            for c in out_string["c_params"]:
                if fn["params"][c]["name"] in missing:
                    missing.remove(fn["params"][c]["name"])
        if missing:
            raise EmitError(f"{method['function']}: unmapped C params {missing}")

        # build the call
        rt = fn["return_type"]
        body = list(pre)
        if out_string:
            buf_i, cap_i = out_string["c_params"]
            inner_args = list(call_args)
            inner_args[buf_i] = "buf_"
            inner_args[cap_i] = "cap_"
            body.append("return dewpy::out_string([&](char *buf_, uint32_t cap_) {")
            body.append(f"  return {fn['name']}({', '.join(inner_args)});")
            body.append("});")
            return_type = ""
        else:
            call = f"{fn['name']}({', '.join(call_args)})"
            if mode in ("init", "static_ctor"):
                target = "self->h" if mode == "init" else "out_.h"
                call_line = f"{target} = {call};"
            elif gated:
                body.append("bool ok_ = false;")
                call_line = f"ok_ = {call};"
            elif return_needed == "value":
                body.append(f"{self._return_decl(rt)} ret_{{}};")
                call_line = f"ret_ = {call};"
            else:
                call_line = f"{call};"
            # constructors release the GIL too: create can do file/network IO,
            # and no Python callbacks can be registered yet, so it is safe
            if method["blocking"] or mode in ("init", "static_ctor"):
                body += ["{", "  nb::gil_scoped_release release_;", f"  {call_line}", "}"]
            else:
                body.append(call_line)

            # error handling
            if method["error_convention"] == "error_out_owned":
                if mode in ("init", "static_ctor"):
                    target = "self->h" if mode == "init" else "out_.h"
                    body.append(f"if (!{target})")
                    body.append(f"  dewpy::throw_consume(error_, \"{fn['name']} failed\");")
                    body.append("if (error_)")
                    body.append("  dew_error_destroy(error_);")
                else:
                    body.append("if (error_)")
                    body.append(f"  dewpy::throw_consume(error_, \"{fn['name']} failed\");")
            elif method["error_convention"] == "error_arg_caller_owned":
                if mode not in ("init", "static_ctor"):
                    raise EmitError(
                        f"{fn['name']}: the caller-owned dew_error_t* convention is only implemented for "
                        f"constructors (this is a {mode}); add a custom binding entry"
                    )
                target = "self->h" if mode == "init" else "out_.h"
                body.append(f"if (!{target})")
                body.append(f"  error_.throw_error(\"{fn['name']} failed\");")

            # return value assembly
            return_type = ""
            if mode == "static_ctor":
                body.append("return out_;")
            elif mode == "init":
                pass
            elif gated:
                return_type = " -> nb::object"
                body.append("if (!ok_)")
                body.append("  return nb::none();")
                if len(result_exprs) == 1:
                    body.append(f"return nb::cast({result_exprs[0][1]});")
                else:
                    body.append("nb::dict result_;")
                    for key, expr in result_exprs:
                        body.append(f'result_["{key}"] = {expr};')
                    body.append("return nb::cast(result_);")
            elif return_needed == "value" and not result_exprs:
                keep = self._token_return(rt)
                body.append("return ret_;")
                if keep:
                    keep_alive_args.append("return")
            elif not result_exprs and return_needed is None:
                pass
            elif len(result_exprs) == 1 and return_needed is None:
                body.append(f"return {result_exprs[0][1]};")
            else:
                body.append("nb::dict result_;")
                if return_needed == "value":
                    body.append('result_["return"] = ret_;')
                for key, expr in result_exprs:
                    body.append(f'result_["{key}"] = {expr};')
                body.append("return result_;")

        # assemble def
        lambda_params = []
        if mode == "init":
            lambda_params.append(f"{holder} *self")
        elif mode == "method":
            lambda_params.append(f"{holder} &self")
        lambda_params += [f"{cpp_type} {cpp_name}" for cpp_type, cpp_name, _, _ in params]

        prologue = []
        if mode == "init":
            prologue.append(f"new (self) {holder}();")
        if mode == "static_ctor":
            prologue.append(f"{holder} out_{{}};")

        def_args = [f'"{py_name}"_a' for _, _, py_name, _ in params]
        if method["annotations"].get("py.no_keep_alive") is True:
            # for un-registering calls, a keep_alive record would pin the
            # argument to the receiver forever -- the opposite of the intent
            keep_alive_args = []
        for pos in keep_alive_args:
            if pos == "return":
                nurse, patient = 0, 1
            elif mode in ("init", "method"):
                nurse, patient = 1, pos + 1
            else:  # static_ctor / free: return keeps args alive
                nurse, patient = 0, pos
            def_args.append(f"nb::keep_alive<{nurse}, {patient}>()")
        doc = _doc_literal(method["doc"])
        if doc:
            def_args.append(doc)

        if mode == "init":
            def_name = '"__init__"'
            def_kind = "def"
        elif mode == "static_ctor":
            def_name = f'"{method["create_suffix"]}"'
            def_kind = "def_static"
        else:
            def_name = f'"{method["bound_name"]}"'
            def_kind = "def"

        lines = [f"  // {fn['name']} ({fn['header']}:{fn['line']})"]
        lines.append(f"  cls.{def_kind}({def_name}, []({', '.join(lambda_params)}){return_type} {{")
        for line in prologue + body:
            lines.append(f"    {line}")
        suffix = (", " + ", ".join(def_args)) if def_args else ""
        lines.append(f"  }}{suffix});")
        return "\n".join(lines)

    def _return_decl(self, rt):
        if rt["kind"] in ("builtin", "enum"):
            return rt["name"]
        if rt["kind"] == "record":
            return rt["name"]
        raise EmitError(f"unsupported return type {rt['spelling']}")

    def _token_return(self, rt):
        return rt["kind"] == "record" and self.struct_kind.get(rt["name"]) == "token"

    # ------------------------------------------------------------------
    # callback registrars
    # ------------------------------------------------------------------

    def _callback_user_ptr_index(self, callback):
        candidates = [
            idx
            for idx, p in enumerate(callback["params"])
            if p["type"]["kind"] == "pointer" and p["type"]["pointee"]["kind"] == "void"
        ]
        if len(candidates) != 1:
            raise EmitError(f"callback {callback['name']}: expected exactly one void* user_ptr param")
        return candidates[0]

    def _callback_param_decl(self, param):
        t = param["type"]
        name = _san(param["name"])
        if t["kind"] == "array":
            const = "const " if t["element"]["const"] else ""
            return f"{const}{t['element']['name']} {name}[{t['extents'][0]}]", name
        return f"{t['spelling']} {name}", name

    def _callback_marshal(self, callback, param, name):
        t = param["type"]
        if t["kind"] in ("builtin", "enum"):
            return name
        if t["kind"] == "array" and t["element"]["kind"] == "builtin":
            items = ", ".join(f"{name}[{k}]" for k in range(t["extents"][0]))
            return f"nb::make_tuple({items})"
        if t["kind"] == "pointer":
            pointee = t["pointee"]
            if pointee["kind"] == "opaque" and pointee["name"] == "dew_error_t":
                return f"dewpy::error_object({name})"
            if pointee["kind"] == "builtin" and pointee["name"] == "char":
                return f"({name} ? nb::object(nb::str({name})) : nb::object(nb::none()))"
        raise EmitError(f"callback {callback['name']}: cannot marshal param '{param['name']}' ({t['spelling']})")

    def _emit_trampoline(self, ctx_name, member, callback):
        if callback["return_type"]["kind"] != "void":
            raise EmitError(f"callback {callback['name']}: non-void callbacks need custom binding")
        user_ptr = self._callback_user_ptr_index(callback)
        decls, names = zip(*(self._callback_param_decl(p) for p in callback["params"]))
        tramp = f"cb_{ctx_name}_{member}"
        py_args = [
            self._callback_marshal(callback, p, names[idx])
            for idx, p in enumerate(callback["params"])
            if idx != user_ptr
        ]
        lines = [
            f"static void {tramp}({', '.join(decls)})",
            "{",
            f"  auto *ctx = static_cast<{ctx_name} *>({names[user_ptr]});",
            "  nb::gil_scoped_acquire gil;",
            "  try",
            "  {",
            f"    ctx->{member}({', '.join(py_args)});",
            "  }",
            "  catch (nb::python_error &e)",
            "  {",
            f"    e.discard_as_unraisable(\"dew callback {member}\");",
            "  }",
            "  catch (...)",
            "  {",
            "    // nothing to report to (this callback has no error out-param), and the",
            "    // caller is C compiled -fno-exceptions: never let anything escape",
            f"    PyErr_SetString(PyExc_RuntimeError, \"C++ exception in dew callback {member}\");",
            "    PyErr_WriteUnraisable(Py_None);",
            "  }",
            "}",
            "",
        ]
        self.blocks.append("\n".join(lines))
        return tramp

    def _emit_callback_method(self, cls, method):
        fn = self.raw_functions[method["function"]]
        holder = self.holders[cls["handle_type"]]
        arg = next(a for a in method["args"] if a["role"] in ("callback_struct", "callback_fn"))
        if len(method["args"]) != 1 or method["results"]:
            raise EmitError(f"{method['function']}: callback registrar with extra args/results needs custom binding")
        if not arg["has_user_ptr"]:
            raise EmitError(f"{method['function']}: registrar without user_ptr needs a custom binding entry")

        ctx_name = f"ctx_{fn['name']}"
        if arg["role"] == "callback_struct":
            struct = self.raw_structs[arg["callback_struct"]]
            members = [(f["name"], self.callbacks[f["type"]["name"]]) for f in struct["fields"]]
        else:
            members = [(arg["bound_name"], self.callbacks[arg["callback"]])]

        ctx_lines = [f"struct {ctx_name} final : dewpy::cb_ctx_base", "{"]
        ctx_lines += [f"  nb::object {name};" for name, _ in members]
        ctx_lines += ["};", ""]
        self.blocks.append("\n".join(ctx_lines))

        trampolines = {name: self._emit_trampoline(ctx_name, name, callback) for name, callback in members}

        struct_i, user_i = arg["c_params"]
        body = [f"auto ctx_owner_ = std::make_unique<{ctx_name}>();"]
        body.append(f"{ctx_name} *ctx_ = ctx_owner_.get();")
        for name, _ in members:
            body.append(f"ctx_->{name} = std::move({_san(name)});")
        if arg["role"] == "callback_struct":
            record = arg["callback_struct"]
            body.append(f"{record} callbacks_{{}};")
            for name, _ in members:
                body.append(f"if (dewpy::callable_set(ctx_->{name}))")
                body.append(f"  callbacks_.{name} = &{trampolines[name]};")
            call_args = [None] * len(fn["params"])
            call_args[method["self_param"]] = "self.h"
            call_args[struct_i] = "callbacks_"
            call_args[user_i] = "ctx_"
        else:
            name = members[0][0]
            call_args = [None] * len(fn["params"])
            call_args[method["self_param"]] = "self.h"
            call_args[struct_i] = f"dewpy::callable_set(ctx_->{name}) ? &{trampolines[name]} : nullptr"
            call_args[user_i] = "ctx_"
        # the holder owns the ctx BEFORE the library learns its address, so no
        # failure path can free a registered context
        body.append("self.ctxs.push_back(std::move(ctx_owner_));")
        body.append(f"{fn['name']}({', '.join(call_args)});")

        lambda_params = [f"{holder} &self"] + [f"nb::object {_san(name)}" for name, _ in members]
        def_args = [f'"{name}"_a = nb::none()' for name, _ in members]
        doc = _doc_literal(method["doc"])
        if doc:
            def_args.append(doc)
        lines = [f"  // {fn['name']} ({fn['header']}:{fn['line']})"]
        lines.append(f"  cls.def(\"{method['bound_name']}\", []({', '.join(lambda_params)}) {{")
        for line in body:
            lines.append(f"    {line}")
        lines.append(f"  }}, {', '.join(def_args)});")
        return "\n".join(lines)

    # ------------------------------------------------------------------
    # classes
    # ------------------------------------------------------------------

    def emit_classes(self):
        custom_by_class = {}
        for fn_name, (bound_class, snippet) in _CUSTOM_CLASS_SNIPPETS.items():
            custom_by_class.setdefault(bound_class, []).append((fn_name, snippet))

        for cls in self.semantic["classes"]:
            if _py_skipped(cls):
                for ctor in cls["constructors"]:
                    self._skip(ctor["function"], f"class {cls['bound_name']} is py.skip")
                if cls["destructor"]:
                    self._skip(cls["destructor"]["function"], f"class {cls['bound_name']} is py.skip")
                for method in cls["methods"]:
                    self._skip(method["function"], f"class {cls['bound_name']} is py.skip")
                continue

            holder = self.holders[cls["handle_type"]]
            bound = cls["bound_name"]
            doc = _doc_literal(cls["doc"])
            doc_arg = f", {doc}" if doc else ""
            lines = [f"static void register_{bound}(nb::module_ &m)", "{"]
            lines.append(f"  auto cls = nb::class_<{holder}>(m, \"{bound}\"{doc_arg});")

            for ctor in cls["constructors"]:
                mode = "init" if ctor["create_suffix"] == "" else "static_ctor"
                lines.append(self._emit_plain_method(cls, ctor, mode))
                self.covered.add(ctor["function"])
            if cls["destructor"]:
                self.covered.add(cls["destructor"]["function"])

            for method in cls["methods"]:
                if _py_skipped(method):
                    self._skip(method["function"], "py.skip")
                    continue
                if method["function"] in _CUSTOM_CLASS_SNIPPETS:
                    continue  # emitted below via the custom registry
                if any(a["role"] in ("callback_struct", "callback_fn") for a in method["args"]):
                    lines.append(self._emit_callback_method(cls, method))
                else:
                    lines.append(self._emit_plain_method(cls, method, "method"))
                self.covered.add(method["function"])

            for fn_name, snippet in custom_by_class.get(bound, []):
                lines.append(f"  // custom binding: {fn_name}")
                lines.append(f"  {snippet}")
                self.covered.add(fn_name)

            lines += ["}", ""]
            self.registers.append("\n".join(lines))
            self.register_calls.append(f"register_{bound}(m);")

    def emit_free_functions(self):
        lines = ["static void register_free_functions(nb::module_ &m)", "{"]
        emitted = False
        for fn in self.semantic["free_functions"]:
            if _py_skipped(fn):
                self._skip(fn["function"], "py.skip")
                continue
            if fn["function"] in _CUSTOM_CLASS_SNIPPETS:
                continue  # covered by a class's custom snippet
            raise EmitError(f"free function {fn['function']}: module-level emission not implemented; add a custom entry")
        lines += ["  (void)m;", "}", ""]
        if not emitted:
            pass
        self.registers.append("\n".join(lines))
        self.register_calls.append("register_free_functions(m);")

    def _skip(self, fn_name, reason):
        self.covered.add(fn_name)
        self.skipped.append((fn_name, reason))

    # ------------------------------------------------------------------
    # top level
    # ------------------------------------------------------------------

    def generate(self):
        self.emit_holders()
        self.emit_enums()
        self.emit_value_structs()
        self.emit_classes()
        self.emit_free_functions()
        self.emit_constants()

        uncovered = [f["name"] for f in self.raw["functions"] if f["name"] not in self.covered]
        # semantic-level bind:skip functions never reach the semantic layer;
        # account for them so coverage is exact
        for name in self.semantic["skipped"]["functions"]:
            if name in uncovered:
                uncovered.remove(name)
                self.skipped.append((name, "bind: skip"))
        if uncovered:
            raise EmitError(f"{len(uncovered)} function(s) neither bound nor skipped: {uncovered}")

        header_includes = "\n".join(f"#include <{h}>" for h in self.headers)
        parts = [
            "// GENERATED by tools/bindgen/generate_nanobind.py -- do not edit.",
            "//",
            "// Python bindings for the dewfall public C API. Regenerated at build",
            "// time from dew_api.json; the custom (hand-written) pieces live in",
            "// bindings/python/custom/.",
            "",
            "#include <nanobind/nanobind.h>",
            "#include <nanobind/ndarray.h>",
            "#include <nanobind/stl/array.h>",
            "#include <nanobind/stl/string.h>",
            "#include <nanobind/stl/string_view.h>",
            "#include <nanobind/stl/vector.h>",
            "",
            "#include <array>",
            "#include <cstring>",
            "#include <memory>",
            "#include <string>",
            "#include <string_view>",
            "#include <utility>",
            "#include <vector>",
            "",
            header_includes,
            "",
            '#include "dew_py_support.h"',
            "",
            "namespace nb = nanobind;",
            "using namespace nb::literals;",
            "",
            "namespace",
            "{",
            "",
        ]
        parts += self.blocks
        parts.append("} // namespace")
        parts.append("")
        parts.append('#include "custom/file_convert_callbacks.h"')
        parts.append('#include "custom/amend.h"')
        parts.append('#include "custom/query.h"')
        parts.append('#include "custom/query_async.h"')
        parts.append("")
        parts.append("namespace")
        parts.append("{")
        parts.append("")
        parts += self.registers
        parts.append("} // namespace")
        parts.append("")
        # The extension is `dew._dew`, re-exported by dew/__init__.py: the
        # package directory is what carries get_include()/get_cmake_dir(), the
        # type stubs and the shipped libraries, and a directory cannot coexist
        # with a top-level dew.<ext>. This name MUST match the built filename
        # or import fails with "dynamic module does not define module export
        # function".
        parts.append("NB_MODULE(_dew, m)")
        parts.append("{")
        parts.append('  m.doc() = "Compiled core of the dewfall point cloud bindings (see the dew package).";')
        parts.append("  dewpy::register_error(m);")
        for call in self.register_calls:
            parts.append(f"  {call}")
        parts.append("}")
        parts.append("")
        return "\n".join(parts)

    def report(self):
        bound = len(self.raw["functions"]) - len(self.skipped)
        print(f"bindgen: bound {bound}/{len(self.raw['functions'])} public functions", file=sys.stderr)
        for name, reason in sorted(self.skipped):
            print(f"bindgen:   skipped {name} ({reason})", file=sys.stderr)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--ir", required=True, help="path to dew_api.json")
    parser.add_argument("--output", required=True, help="path to write the generated .cpp")
    args = parser.parse_args()

    with open(args.ir, "r", encoding="utf-8") as f:
        document = json.load(f)
    if document.get("ir_version") != 1:
        sys.exit(f"error: unsupported ir_version {document.get('ir_version')}")

    emitter = Emitter(document)
    try:
        text = emitter.generate()
    except EmitError as e:
        sys.exit(f"bindgen error: {e}")

    with open(args.output, "w", encoding="utf-8") as f:
        f.write(text)
    emitter.report()
    print(f"bindgen: wrote {args.output}", file=sys.stderr)


if __name__ == "__main__":
    main()
