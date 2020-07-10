
# Config

Install cross platform compiler [gcc-linaro-6.5.0](https://releases.linaro.org/components/toolchain/binaries/6.5-2018.12/aarch64-linux-gnu/gcc-linaro-6.5.0-2018.12-x86_64_aarch64-linux-gnu.tar.xz)

```bash
GCCARM_LINK=https://releases.linaro.org/components/toolchain/binaries/6.5-2018.12/aarch64-linux-gnu/gcc-linaro-6.5.0-2018.12-x86_64_aarch64-linux-gnu.tar.xz
wget -O /tmp/gcc-linaro-6.5.0-2018.12-x86_64_aarch64-linux-gnu.tar.xz ${GCCARM_LINK}  
tar xf /tmp/gcc-linaro-6.5.0-2018.12-x86_64_aarch64-linux-gnu.tar.xz --strip-components=1 -C /opt/  
rm -rf /tmp/gcc-linaro-6.5.0-2018.12-x86_64_aarch64-linux-gnu.tar.xz
```

Then set `cmake_c_compiler`  `cmake_cxx_compiler`  `cmake_find_root_path`  in build.properties.local

# Compile
    
```bash
mkdir build 
cd build
cmake ..
make
```

# Run example and unittest

TODO

# Code Contributing

see [CONTRIBUTING.md](./CONTRIBUTING.md)
