# draco_encode_cpp

Compiling and running instructions

1. Make sure draco is installed and you know the path to draco
2. Run the following commands

    `$mkdir build` 
    
    `$cd build`
  
    `$cmake ..` 
  
    `$make`

3. You should see the executable `draco_encode` inside the build folder.

note:
Do "make install" in draco source build folder - /home/allan/draco/build

./draco_decoder -i in.drc -o out.obj

when building open3d from source:
cmake -DGLIBCXX_USE_CXX11_ABI=ON ..
^ flag very important or else external projects won't link properly