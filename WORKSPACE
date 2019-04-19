load("@bazel_tools//tools/build_defs/repo:git.bzl", "git_repository", "new_git_repository")
load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")

# googletest contains a BUILD file
git_repository(
    name = "gtest",
    remote = "https://github.com/google/googletest",
    tag = "release-1.8.1"
)

git_repository(
    name = "com_github_nelhage_rules_boost",
    commit = "6d6fd834281cb8f8e758dd9ad76df86304bf1869",
    remote = "https://github.com/nelhage/rules_boost",
)

load("@com_github_nelhage_rules_boost//:boost/boost.bzl", "boost_deps")
boost_deps()

# libuWS
new_local_repository(
    name = "uWebSocket",
    path = "/usr/",
    build_file = "bazel/uWS.BUILD",
)

