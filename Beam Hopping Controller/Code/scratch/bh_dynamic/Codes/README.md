# Beam-hopping source locations

Only these files in this directory are build inputs:

- `sat-bh-example.cc`
- `sat-bh-2d-footprint.cc`
- `sat-constellation-params.h`
- `CMakeLists.txt`

The beam-hopping implementation belongs in:

```text
contrib/satellite/helper/
```

Files such as `sat-bh-helper.cc`, `sat-bh-scheduler.cc`, and their headers in
this directory are copies only. CMake does not compile them. Editing or
overwriting those copies will not change the executable.

Build and run the unambiguous target from the repository root:

```bash
./ns3 build bh_dynamic/Codes/sat-bh-example
./ns3 run "bh_dynamic/Codes/sat-bh-example --scenario=starlink25"
```

Or use:

```bash
./run-starlink25.sh
```

That launcher always rebuilds the exact target before executing it.
