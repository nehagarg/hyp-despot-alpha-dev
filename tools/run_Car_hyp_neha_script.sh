t=$1
n=$2
solv=$3

      dir="HypDespot_Car_Driving_Results_$solv/t$t/n$n";
      echo $dir
      mkdir -p $dir;
      for i in {1..1000}; do
            echo "run HYP "-n $n
            random_num=$RANDOM
            ../Release/devel/lib/hyp_despot/hyp_despot_CarDriving --runs 1 -s 200 -t $t -r $random_num -n $n --GPU 0 --CPU 0 --solver=$solv &> $dir/trial_$i.log
            echo "End "$i
      done;
echo "End car driving experiments"

