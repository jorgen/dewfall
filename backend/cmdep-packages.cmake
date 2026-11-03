# Dependencies of dew_server (the dewfall.limilind.com backend), fetched by cmake-dep (CmDepFetch() in CMakeLists.txt). Each also
# declares its own dependencies; cmdep.lock records the whole resolved graph. After changing
# a pin here run `cmdep lock` (cmdep is in bin/ of github.com/jorgen/cmake-dep) and commit both.

CmDepFetchGit(vio 6dde1b6
    https://github.com/jorgen/vio.git
    6dde1b6d3b0cd1092bfaccf4a201f0d723aa237a
    SHA256=0aed5201b64fa1aaf25287f105c13a4f1aac5307b12ffc4d6761d3aef8a3fcbb)

CmDepFetchPackage(structify b114582
    https://github.com/jorgen/structify/archive/b1145826ab4a06cbe32ae0c373a3161f74c48ac8.tar.gz
    SHA256=49f3af94194bd37aa76d99c1d3e351eb237b838ae75d21fecaf6ac4a806c7606)

CmDepFetchGit(prism adc2a21
    https://github.com/jorgen/prism.git
    adc2a21b1717d3ee32f72575b49febd830962bbe
    SHA256=42462ddab18a08be4897f77b9432fd0583e522c3e43d9350837dc9d187a40a06)

