for dirName in $(ls -p); do
  pathToRsdf=$(find $dirName -name 'model.rsdf')
  if [ -n "$pathToRsdf" ]
  then
    echo "\$pathToRsdf = $pathToRsdf"
    erb $pathToRsdf > ./$dirName/model.sdf
  fi
done

for erbWorldFile in $(ls *.world.erb); do
  erb ./$erbWorldFile > ./${erbWorldFile%.erb}
done
