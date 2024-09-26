echo "#pragma once" > shaders.cxx
echo "#include <string>" >> shaders.cxx
echo "//=============================" >> shaders.cxx

echo "const std::string objVS = R\"(" >> shaders.cxx
cat shaderObj.vs >>shaders.cxx
echo ")\";" >>shaders.cxx
echo "//=============================" >> shaders.cxx

echo "const std::string objFS = R\"(" >> shaders.cxx
cat shaderObj.fs >>shaders.cxx
echo ")\";" >>shaders.cxx
echo "//=============================" >> shaders.cxx

echo "const std::string markerVS = R\"(" >> shaders.cxx
cat shaderMarker.vs >>shaders.cxx
echo ")\";" >>shaders.cxx
echo "//=============================" >> shaders.cxx

echo "const std::string markerFS = R\"(" >> shaders.cxx
cat shaderMarker.fs >>shaders.cxx
echo ")\";" >>shaders.cxx
echo "//=============================" >> shaders.cxx

echo "const std::string shadowVS = R\"(" >> shaders.cxx
cat shaderShadow.vs >>shaders.cxx
echo ")\";" >>shaders.cxx
echo "//=============================" >> shaders.cxx

echo "const std::string shadowFS = R\"(" >> shaders.cxx
cat shaderShadow.fs >>shaders.cxx
echo ")\";" >>shaders.cxx
echo "//=============================" >> shaders.cxx

echo "const std::string textVS = R\"(" >> shaders.cxx
cat shaderText.vs >>shaders.cxx
echo ")\";" >>shaders.cxx
echo "//=============================" >> shaders.cxx

echo "const std::string textFS = R\"(" >> shaders.cxx
cat shaderText.fs >>shaders.cxx
echo ")\";" >>shaders.cxx
echo "//=============================" >> shaders.cxx
