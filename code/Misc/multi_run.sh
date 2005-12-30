
# Usage: multi_run OutDir numRuns numP
outDir=$1
numRuns=$2
numP=$3
workDir=`pwd`
lasFile=$workDir/las.log
odoFile=$workDir/odo.log
seedOffset=100

echo    Output Directory: $outDir
echo      Number of Runs: $numRuns
echo Number of Particles: $numP
echo   Current Directory: $workDir
echo   Seed Offset      : $seedOffset
echo Laser: $lasFile
echo Odo  : $odoFile

if [ -e $outDir ]
then
  echo Directory exists.
else
  mkdir $outDir
fi

for ((  i = 1 ;  i <= $numRuns;  i++  ))
do
  dirOut=`printf "%s/%03d" $outDir $i`

  if [ -e $dirOut ]
  then
    echo Directory exists
  else
    echo Creating Directory: $dirOut
    mkdir $dirOut
  fi

  echo Changing to: $dirOut
  cd $dirOut

  seed=`expr $i + $seedOffset`
  echo Run SLAM: slam $odoFile $lasFile dd $numP $seed
  slam $odoFile $lasFile dd $numP $seed > out

  cd $workDir

done





