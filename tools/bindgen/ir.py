"""Shared schema constants for the dewfall binding IR (dew_api.json).

The IR has two layers:
  raw      -- faithful libclang extraction of the public C headers
  semantic -- derived object-oriented view (classes, methods, roles) that
              every generator consumes; it never mentions a target language

Bump IR_VERSION when the schema shape changes incompatibly.
"""

IR_VERSION = 1

# --- argument/result roles in the semantic layer -------------------------
# An "arg" is one bound parameter; it may consume several C params
# (recorded in c_params, indices into the raw function's param list).
ROLE_SELF = "self"
ROLE_SCALAR = "scalar"
ROLE_ENUM = "enum"
ROLE_STRING_IN = "string_in"          # const char* + length param
ROLE_ARRAY_IN = "array_in"            # fixed-extent input array
ROLE_ARRAY_OUT = "array_out"          # fixed-extent output array -> result
ROLE_OUT_PARAM = "out_param"          # pointer-to-scalar/enum output -> result
ROLE_OUT_STRING = "out_string"        # caller char buffer + size, return = length
ROLE_STRUCT_IN = "struct_in"          # record by value or by const pointer
ROLE_STRUCT_OUT = "struct_out"        # pointer-to-record filled by callee
ROLE_BUFFER_ARRAY_IN = "buffer_array_in"  # ptr + count of records ("arrays:" tag)
ROLE_CALLBACK_STRUCT = "callback_struct"  # by-value struct of fn pointers (+ user_ptr)
ROLE_CALLBACK_FN = "callback_fn"      # bare fn pointer (+ user_ptr)
ROLE_HANDLE_IN = "handle_in"          # pointer to another bound handle class
ROLE_ERROR_OUT = "error_out"          # dew_error_t** (hidden, raises on failure)
ROLE_ERROR_IN = "error_in"            # caller-owned dew_error_t* (hidden)

# --- error conventions ----------------------------------------------------
ERROR_NONE = "none"
ERROR_OUT_OWNED = "error_out_owned"            # dew_error_t** out-param, caller owns result
ERROR_ARG_CALLER_OWNED = "error_arg_caller_owned"  # caller passes a dew_error_t*

# --- sanity tripwire --------------------------------------------------------
# Updated deliberately whenever the public API grows; a mismatch is a loud
# warning from parse_headers.py and an assertion in test_ir.py.
EXPECTED_COUNTS = {
    "functions": 159,
    "opaque_types": 19,
    "enums": 21,
    "structs": 28,
    "callbacks": 25,
    "macro_constants": 19,
}

# Annotation keys the parser understands. Unknown keys warn and pass through;
# "py."/"cpp." prefixes are consumer-specific and always pass through silently.
KNOWN_ANNOTATIONS = {
    "bind",        # bind: skip
    "class",       # class: dew_xxx_t
    "rename",      # rename: name
    "arrays",      # arrays: ptr[count], other[count2]
    "string",      # string: ptr[len]
    "out_string",  # out_string: buf[size]
    "out",         # out: a, b
    "in",          # in: a, b
    "nullable",    # nullable: a, b
    "returns",     # returns: new|borrowed
    "blocking",    # blocking
    "awaitable",   # awaitable: poll=<fn> pending=<enum constant> [release=<fn>]
    "destroy",     # destroy: <fn>   -- when the destructor is not named *_destroy
}
