mkdir %~dp0..\src\main\java
protoc --proto_path=%cd%\Protos --java_out=%~dp0..\src\main\java %cd%\Protos\*.proto