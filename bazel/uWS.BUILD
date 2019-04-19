cc_library(
    name = "uWS",
    srcs = ["lib/libuWS.so"],
    hdrs = ["include/uWS/uWS.h", "include/uWS/uUV.h"],
    linkopts = [
        "-lz",
        "-lssl",
        "-lcrypto"
    ],
    visibility = ["//visibility:public"],
)