# VNaoBridge
VNaoBridge takes care of the communication between V-Rep and NAOqi-bin.

## Prerequisites
VNaoBridge requires a V-Rep (tested with 3.5.0 on Linux and 3.6.1 on Windows) installation and the NAOqi Simulator SDK (tested with >=2.1.2.17).
Compilation requires a current GCC (Linux) or MSVC 2010 Express (Windows).

## Compilation

### Linux
```
$ mkdir build
$ cd build
$ cmake -DVREP_PATH="~/NAO/V-REP_PRO_EDU_V3_5_0_Linux" -DSIMSDK_PATH="~/NAO/simulator-sdk-2.1.4.1-linux64" ..
$ make
```

### Windows

Use cmd.exe to run the following commands from the VNaoBridge root:
```
$ @call "%VS100COMNTOOLS%/vsvars32.bat"
$ mkdir build
$ cd build
$ cmake -G "Visual Studio 10 2010" -DVREP_PATH="C:\Program Files\V-REP3\V-REP_PRO_EDU" -DSIMSDK_PATH="C:\Program Files\V-REP3\V-REP_PRO_EDU\simulator-sdk" ..
$ msbuild vnaobridge.sln /p:Configuration=Release
```

### MacOS

Install gcc using `brew install gcc` and then run:

```
$ mkdir build
$ cd build
$ cmake -DVREP_PATH=~/NAO/V-REP_PRO_EDU_V3_5_0_Mac -DSIMSDK_PATH=~/NAO/V-REP_PRO_EDU_V3_5_0_Mac/simulator-sdk -DCMAKE_CXX_COMPILER=g++-8 ..
$ make
```

## Running

Rename the simulator sdk folder to "simulator-sdk" and place it inside your V-Rep folder.
Place the vnaobridge executable inside "simulator-sdk/bin".

Then VNaoBridge can be started from the NAO Lua Script using:
```
sim.launchExecutable('simulator-sdk/bin/vnaobridge', '9559')
```
