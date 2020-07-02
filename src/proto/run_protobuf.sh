#cd watrix/proto
protoc_exec=$(which protoc)
echo "compiling proto files ..."
${protoc_exec} --version
${protoc_exec} --cpp_out=. ./*.proto
ls -al 
