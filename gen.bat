echo off
if [%1] == [] goto err
thrift-0.9.2.exe -out %1\gen-cpp --gen cpp %1\Thrift\AFC.thrift
goto ok
:err
echo Missing version parameter
:ok