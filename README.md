# thrust-vector-control
Calculate optimal magnitude and direction of thrust for a controllable rocket flight path

## Setup
- Install [bazel](https://docs.bazel.build/versions/main/install.html), which we use to build and test code

## Norms
- All C++ code must obey the [Google Style Guide](https://google.github.io/styleguide/cppguide.html)
- C++ code must be formatted with `clang-format`
    - Configure your editor to auto-format, use `clang-format -i <files>`, or use `git clang-format`
    - Do `clang-format --help` to learn how to use it if needed
- BUILD files must be formatted with [buildifier](https://github.com/bazelbuild/buildtools/blob/master/buildifier/README.md)
