# tri_exp

`tri_exp` contains the minimal source package and SOP for deploying the
beam-hopping experiment into ns-3.43 with the SNS3 satellite contrib module.

Start here:

```text
INSTALLATION.md
```

Important files:

```text
INSTALLATION.md                         Chinese installation SOP
INSTALL_MANIFEST.txt                    Minimal file manifest
scripts/install_tri_exp_minimal.sh      Dry-run/apply installer
contribsatellite/CMakeLists.txt.txt     Installed as contrib/satellite/CMakeLists.txt
scratch/bh_dynamic/                     Experiment targets
```

Recommended flow:

```bash
cd /home/lucy
cp -a ns-allinone-3.43 ns-allinone-3.43-tri-exp-test
cd /home/lucy/ns-allinone-3.43-tri-exp-test/ns-3.43
rm -rf build cmake-cache

/path/to/tri_exp/scripts/install_tri_exp_minimal.sh --ns3 "$PWD"
/path/to/tri_exp/scripts/install_tri_exp_minimal.sh --ns3 "$PWD" --apply

./ns3 configure --enable-examples --enable-tests
./ns3 build
./ns3 run "bh_dynamic/Codes/sat-bh-example --scenario=starlink25"
./ns3 run "bh_dynamic/Codes/sat-bh-2d-footprint --simTime=0"
```

Do not copy the whole package directly over ns-3. Use the manifest-driven
installer so only the required files are installed.
