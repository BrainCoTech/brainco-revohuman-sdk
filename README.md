# RevoHuman C SDK

C library for RevoHuman data gloves: connects to the gloves, reads the 21 joint angles and full-finger tactile data of
each hand, and computes hand forward kinematics (FK) and the position and orientation of every tactile point on the hand
inside the library.
The interface is C99 and can be included from C++ as-is. It suits programs without a Python runtime and embedded devices.

For tracker poses, wrist cameras, MCAP recording and ROS 2 nodes, use the Python SDK on the
[main branch](https://github.com/BrainCoTech/brainco-revohuman-sdk/tree/main) of this repository.
The two share identical error code values, and the local broadcast of the C library can feed the Python SDK's viewer
directly.

This branch holds only the public headers, a minimal example and the license. The library ships as binary packages,
without source code.

```
LICENSE                     full text of the Apache-2.0 license
include/revohuman.h         glove, tactile, two-hand pairing and kit interfaces
include/revohuman_fk.h      hand forward kinematics, tactile point layout
example/                    minimal example that runs without a glove
```

All interfaces and conventions are documented in the comments of the two headers.

---

## Supported hardware and systems

| Item | Requirement |
|---|---|
| Gloves | RevoHuman P1 or PTV1, one left and one right, connected to the host over USB |
| Architecture | x86_64 / aarch64 |
| System | Linux, glibc ≥ 2.34, libstdc++ providing GLIBCXX ≥ 3.4.30 (met by Ubuntu 22.04 and later) |
| Permissions | The user running the program needs access to the serial port (usually by being in the `dialout` group) |

## Download and verification

Download the release files from
[GitHub Release c-sdk-v0.1.3](https://github.com/BrainCoTech/brainco-revohuman-sdk/releases/tag/c-sdk-v0.1.3):

| File | Contents |
|---|---|
| `revohuman-c-sdk-0.1.3-linux-x86_64.tar.gz` | x86_64 binary package |
| `revohuman-c-sdk-0.1.3-linux-aarch64.tar.gz` | aarch64 binary package |
| `SHA256SUMS` | checksums |

```bash
sha256sum -c SHA256SUMS --ignore-missing
tar -xzf revohuman-c-sdk-0.1.3-linux-$(uname -m).tar.gz
```

Below, the extracted directory is called `<SDK>`; it can be placed anywhere. The `README.md` inside the package lists
every file, `SHA256SUMS` verifies each file, and `BUILDINFO.txt` records the build provenance.

## Usage

**CMake projects**

```cmake
find_package(revohuman 0.1 CONFIG REQUIRED)
target_link_libraries(app PRIVATE revohuman::revohuman)          # shared library
# or target_link_libraries(app PRIVATE revohuman::revohuman_static)  # static library
```

Pass `-DCMAKE_PREFIX_PATH=<SDK>` when configuring. 0.x versions are compatible by minor version:
`find_package(revohuman 0.1)` only accepts 0.1.y. Plain C projects using the static library do not need to add
`-lstdc++` themselves; the package configuration adds it.

**Without CMake**

```bash
cc  app.c   -I<SDK>/include -L<SDK>/lib -lrevohuman
c++ app.cpp -I<SDK>/include -L<SDK>/lib -lrevohuman
```

**Finding the shared library at run time**

For programs linked against the shared library, run this before starting them:

```bash
export LD_LIBRARY_PATH=<SDK>/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}
```

Or copy `librevohuman.so*` next to the program and link with `-Wl,-rpath,'$ORIGIN'`. Programs linked against the static
library do not need this step.

## Try the example

```bash
cmake -S example -B /tmp/rh_example -DCMAKE_PREFIX_PATH=<SDK>
cmake --build /tmp/rh_example
LD_LIBRARY_PATH=<SDK>/lib /tmp/rh_example/rh_info
/tmp/rh_example/rh_info_static
```

Both programs print the version, the hand model fingerprint and the position of the last link in the joint table for
all-zero joint angles, and exit with code 0.
`share/revohuman/example/` in the binary package is the same example.

## License

Apache-2.0, see [LICENSE](LICENSE).
