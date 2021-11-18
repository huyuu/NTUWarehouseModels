
for dirName in $(ls); do
  if grep "model.rsdf" $(ls ./$dirName)
    echo "./$dirName/model.rsdf"
    # erb ./$dirName/model.rsdf > ./$dirName/model.sdf
  fi
done

for erbWorldFile in $(ls *.world.erb); do
  echo "./${erbWorldFile%.erb}"
  # erb ./$erbWorldFile > ./${erbWorldFile%.erb}
done
