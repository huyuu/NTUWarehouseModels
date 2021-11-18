
for dirName in $(ls); do
  if $(find $dirName -name 'model.rsdf')
  then
    echo $(find $dirName -name 'model.rsdf')
    # erb $pathToRsdf > ./$dirName/model.sdf
  fi
done

for erbWorldFile in $(ls *.world.erb); do
  echo "./${erbWorldFile%.erb}"
  # erb ./$erbWorldFile > ./${erbWorldFile%.erb}
done
