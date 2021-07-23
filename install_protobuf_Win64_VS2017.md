# Install protobuf for Win64 and Visual Studio 2017

1. Create the following batch script (e.g. as build-protobuf.bat) and run with e.g. cmd.exe:
    ```powershell
    cd C:/tmp
    if exist protobuf-3.14.0 rd /s /q protobuf-3.14.0
    curl -LO https://github.com/protocolbuffers/protobuf/releases/download/v3.14.0/protobuf-cpp-3.14.0.zip

    Call :UnZipFile "C:\tmp\" "c:\tmp\protobuf-cpp-3.14.0.zip"

    cd protobuf-3.14.0/cmake
    if exist build rd /s /q build
    mkdir build
    cd build

    C:\Progra~1\CMake\bin\cmake.exe ^
        -G "Visual Studio 15 2017" ^
        -DCMAKE_INSTALL_PREFIX:PATH=C:/protobuf-3.14 ^
        -DCMAKE_SHARED_LINKER_FLAGS:STRING=/machine:x64 ^
        -DCMAKE_STATIC_LINKER_FLAGS:STRING=/machine:x64 ^
        -DCMAKE_EXE_LINKER_FLAGS:STRING=/machine:x64 ^
        -DCMAKE_MODULE_LINKER_FLAGS:STRING=/machine:x64 ^
        -DCMAKE_GENERATOR_PLATFORM:INTERNAL=x64 ^
        -Dprotobuf_WITH_ZLIB:BOOL=OFF ^
        -Dprotobuf_BUILD_TESTS:BOOL=OFF ^
        ..
      
    :UnZipFile <ExtractTo> <newzipfile>
    set vbs="%temp%\_.vbs"
    if exist %vbs% del /f /q %vbs%
    >%vbs%  echo Set fso = CreateObject("Scripting.FileSystemObject")
    >>%vbs% echo If NOT fso.FolderExists(%1) Then
    >>%vbs% echo fso.CreateFolder(%1)
    >>%vbs% echo End If
    >>%vbs% echo set objShell = CreateObject("Shell.Application")
    >>%vbs% echo set FilesInZip=objShell.NameSpace(%2).items
    >>%vbs% echo objShell.NameSpace(%1).CopyHere(FilesInZip)
    >>%vbs% echo Set fso = Nothing
    >>%vbs% echo Set objShell = Nothing
    cscript //nologo %vbs%
    if exist %vbs% del /f /q %vbs%
    ```
2. Run extract_includes.bat located in build directory from above
3. Open protobuf.sln located in build directory from above with **Visual Studio 2017**
4. Change Properties->C/C++->Code Generation of **all** solutions for Configuration *Release* AND *Debug*<br>
   - Runtime Library for *Release* must be set to "Multi-threaded DLL (/MD)":<br>
     <img src="https://gitlab.com/tuda-fzd/perception-sensor-modeling/reflection-based-lidar-object-model/uploads/38c646d7442362bbbbce936d67a26836/ASM_protobuf_runtime_library_release.png"  width="800"><br><br>
   - Runtime Library for *Debug* must be set to "Multi-threaded Debug DLL (/MDd)":<br>
     <img src="https://gitlab.com/tuda-fzd/perception-sensor-modeling/reflection-based-lidar-object-model/uploads/be2f9e3f0cda9da9bb4686805379b37e/ASM_protobuf_runtime_library_debug.png"  width="800">
5. Build Solution for *Release* AND for *Debug*
6. Build INSTALL for *Release* AND for *Debug*
