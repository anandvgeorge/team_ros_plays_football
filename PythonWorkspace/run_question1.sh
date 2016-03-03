export VREP_PATH="/home/ryan/apps/V-REP_PRO_EDU_V3_2_3_rev4_64_Linux/vrep.sh"
export ENV_PATH="/home/ryan/Documents/studyaway/DistAutoRobo/team_ros_plays_football/Project_PlayingField_Q1.ttt"
python question1.py &
bash $VREP_PATH $ENV_PATH -gREMOTEAPISERVERSERVICE_19999_FALSE_FALSE
