pushd vendored\emsdk
call emsdk.bat activate latest
popd
call emcmake cmake -S . -B build_web
call cmake --build build_web
