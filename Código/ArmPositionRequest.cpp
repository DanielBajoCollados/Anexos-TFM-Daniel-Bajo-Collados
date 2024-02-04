double MotorsPos[6];
camera.send_request_for_position(MotorsPos);
for(int i=0;i<6;i++)
	coupler.set_goal_len(i, MotorsPos[i]);