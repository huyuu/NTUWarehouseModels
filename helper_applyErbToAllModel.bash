for dirName in $(ls); do
  if find $dirName -name 'model.rsdf'
  then
    pathToRsdf=$(find $dirName -name 'model.rsdf')
    echo "\%pathToRsdf = $pathToRsdf"
    erb $pathToRsdf > ./$dirName/model.sdf
  fi
done

for erbWorldFile in $(ls *.world.erb); do
  erb ./$erbWorldFile > ./${erbWorldFile%.erb}
done
