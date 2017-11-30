./motion_estimator_test /home/luiz/Documentos/Faculdade/Lab/pesquisa/datasets/rgbd_dataset_freiburg1_xyz/index.txt
mkdir resultados/icp/klt/rgbd_dataset_freiburg1_xyz
./media tempo_frame.txt > media.txt
mv media.txt resultados/icp/klt/rgbd_dataset_freiburg1_xyz

./evaluate_rpe.py --verbose --fixed_delta /home/luiz/Documentos/Faculdade/Lab/pesquisa/datasets/rgbd_dataset_freiburg1_xyz/groundtruth.txt pos_relativa.txt > resultados/icp/klt/rgbd_dataset_freiburg1_xyz/vo_analisys.txt

./motion_estimator_test /home/luiz/Documentos/Faculdade/Lab/pesquisa/datasets/rgbd_dataset_freiburg2_xyz/index.txt
mkdir resultados/icp/klt/rgbd_dataset_freiburg2_xyz
./media tempo_frame.txt > media.txt
mv media.txt resultados/icp/klt/rgbd_dataset_freiburg2_xyz

./evaluate_rpe.py --verbose --fixed_delta /home/luiz/Documentos/Faculdade/Lab/pesquisa/datasets/rgbd_dataset_freiburg2_xyz/groundtruth.txt pos_relativa.txt > resultados/icp/klt/rgbd_dataset_freiburg2_xyz/vo_analisys.txt

./motion_estimator_test /home/luiz/Documentos/Faculdade/Lab/pesquisa/datasets/rgbd_dataset_freiburg1_plant/index.txt
mkdir resultados/icp/klt/rgbd_dataset_freiburg1_plant
./media tempo_frame.txt > media.txt
mv media.txt resultados/icp/klt/rgbd_dataset_freiburg1_plant

./evaluate_rpe.py --verbose --fixed_delta /home/luiz/Documentos/Faculdade/Lab/pesquisa/datasets/rgbd_dataset_freiburg1_plant/groundtruth.txt pos_relativa.txt > resultados/icp/klt/rgbd_dataset_freiburg1_plant/vo_analisys.txt

./motion_estimator_test /home/luiz/Documentos/Faculdade/Lab/pesquisa/datasets/rgbd_dataset_freiburg1_teddy/index.txt
mkdir resultados/icp/klt/rgbd_dataset_freiburg1_teddy
./media tempo_frame.txt > media.txt
mv media.txt resultados/icp/klt/rgbd_dataset_freiburg1_teddy

./evaluate_rpe.py --verbose --fixed_delta /home/luiz/Documentos/Faculdade/Lab/pesquisa/datasets/rgbd_dataset_freiburg1_teddy/groundtruth.txt pos_relativa.txt > resultados/icp/klt/rgbd_dataset_freiburg1_teddy/vo_analisys.txt

