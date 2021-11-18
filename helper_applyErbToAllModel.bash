
for dirName in $(ls); do
  pathToRsdf="./$(find $dirName -name 'model.rsdf')"
  if pathToRsdf
    echo $pathToRsdf
    # erb $pathToRsdf > ./$dirName/model.sdf
  fi
done

for erbWorldFile in $(ls *.world.erb); do
  echo "./${erbWorldFile%.erb}"
  # erb ./$erbWorldFile > ./${erbWorldFile%.erb}
done
