for dirName in $(ls); do
  if pathToRsdf="./$(find $dirName -name 'model.rsdf')"
  then
    erb $pathToRsdf > ./$dirName/model.sdf
  fi
done

for erbWorldFile in $(ls *.world.erb); do
  erb ./$erbWorldFile > ./${erbWorldFile%.erb}
done
