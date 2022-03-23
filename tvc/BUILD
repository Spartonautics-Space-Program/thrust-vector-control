cc_library(
    name = "tvc_lib",
    srcs = [
        "tvc.cc",
    ],
    hdrs = [
        "tvc.h",
    ],
    deps = [
        "//linalg:matrix_lib",
        "//util:util_lib",
    ],
)

cc_binary(
    name = "tvc",
    srcs = [
        "tvc_main.cc",
    ],
    deps = [
        ":tvc_lib",
    ],
)

cc_test(
    name = "tvc_test",
    srcs = [
        "tvc_test.cc",
    ],
    deps = [
        ":tvc_lib",
        "//util:util_lib",
    ],
)
