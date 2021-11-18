for dirName in $(ls -p); do
  pathToRsdf=$(find $dirName -name 'model.rsdf')
  if [ -n "$pathToRsdf" ]
  then
    erb $pathToRsdf > ./$dirName/model.sdf
    echo "converted $pathToRsdf to $dirName/model.sdf"
  fi
done

for erbWorldFile in $(ls *.world.erb); do
  erb ./$erbWorldFile > ./${erbWorldFile%.erb}
  echo "converted $erbWorldFile to ${erbWorldFile%.erb}"
done
