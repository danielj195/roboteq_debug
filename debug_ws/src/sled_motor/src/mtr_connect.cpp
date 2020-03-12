#include "sled_motor/mtr_connect.h"

Mtr_Connect::Mtr_Connect(std::string port_name)
{
	cmd_sled_pos_sub = nh_.subscribe<arm_sanding_utilities::SledPosition>("/sled/cmd_position", 10, &Mtr_Connect::cmd_pos_callback, this);
	//sled_calib_sub = nh_.subscribe<arm_sanding_utilities::SledCalibration>("/sled/calibration", 10, &Mtr_Connect::calib_callback, this);
	plan_complete_sub = nh_.subscribe<arm_sanding_utilities::PlanCompleteStatus>("/sensors/planstatus", 10, &Mtr_Connect::complete_callback, this);
	//dest_reached_pub = nh_.advertise<arm_sanding_utilities::SledDestReached>("/sled/dest_reached", 10);
	RoboteqDevice device();
	port = port_name;
}

void Mtr_Connect::make_connection()
{
	int status = device.Connect(port);
	if (status != RQ_SUCCESS)
		{
			cout << "Error connecting to device: " << status << "." << endl;
		}
	this->default_config();
	while(ros::ok() && !plan_complete){
		ros::spinOnce();
	}
	device.Disconnect();
}

void Mtr_Connect::move2pos(int position)
{	
	cout << "- Set Operating Mode: SetConfig(_MMOD, 1, "<<0<<")...";  //set mode to open loop mode
	if ((status = device.SetConfig(_MMOD, 1, 0)) != RQ_SUCCESS)
		cout << "failed --> " << status << endl;
	else
		cout << "succeeded." << endl;
	sleepms(20);

	cout << "- Set Position: SetCommand(_C, 1, nn)...";            //set encoder count value to 0
	if ((status = device.SetCommand(_C, 1, 0)) != RQ_SUCCESS)
		cout << "failed --> " << status << endl;
	else
		cout << "succeeded." << endl;
	sleepms(10);

	cout << "- Set Operating Mode: SetConfig(_MMOD, 1, "<<3<<")...";  //reset mode back to count position mode
	if ((status = device.SetConfig(_MMOD, 1, 3)) != RQ_SUCCESS)
		cout << "failed --> " << status << endl;
	else
		cout << "succeeded." << endl;
	sleepms(10);

	cout << "- Set Operating Mode: SetConfig(_MMOD, 2, "<<0<<")...";  //set mode to open loop mode
	if ((status = device.SetConfig(_MMOD, 2, 0)) != RQ_SUCCESS)
		cout << "failed --> " << status << endl;
	else
		cout << "succeeded." << endl;
	sleepms(10);

	cout << "- Set Position: SetCommand(_C, 2, nn)...";            //set encoder count value to 0
	if ((status = device.SetCommand(_C, 2, 0)) != RQ_SUCCESS)
		cout << "failed --> " << status << endl;
	else
		cout << "succeeded." << endl;
	sleepms(20);

	cout << "- Set Operating Mode: SetConfig(_MMOD, 2, "<<3<<")...";  //reset mode back to count position mode
	if ((status = device.SetConfig(_MMOD, 2, 3)) != RQ_SUCCESS)
		cout << "failed --> " << status << endl;
	else
		cout << "succeeded." << endl;
	sleepms(10);
	std::string str = "!P 1 9000_!P 2 9000_";
	device.Write(str);
	device.ReadAll(str);

	// cout << "- Set Position: SetCommand(_P, 1, nn)...";
	// if ((status = device.SetCommand(_P, 1, position)) != RQ_SUCCESS)   //36212
	// 	cout << "failed --> " << status << endl;
	// else
	// 	cout << "succeeded." << endl;
	// // sleepms(10);

	// cout << "- Set Position: SetCommand(_P, 2, nn)...";
	// if ((status = device.SetCommand(_P, 2, position)) != RQ_SUCCESS)   //36212
	// 	cout << "failed --> " << status << endl;
	// else
	// 	cout << "succeeded." << endl;
	// sleepms(10);
	// status = device.SetCommand(_P, 1, position);
	// status = device.SetCommand(_P, 2, position);

	ros::Duration(5).sleep();

	int result;
	cout << "- Read Encoder 1 Count: GetValue(_C, 1)...";
	if ((status = device.GetValue(_C, 1, result)) != RQ_SUCCESS)
		cout << "failed --> " << status << endl;
	else
		cout << "Encoder 1 Count --> " << result << endl;
	sleepms(10);

	cout << "- Read Encoder 2 Count: GetValue(_C, 2)...";
	if ((status = device.GetValue(_C, 2, result)) != RQ_SUCCESS)
		cout << "failed --> " << status << endl;
	else
		cout << "Encoder 2 Count --> " << result << endl;
	sleepms(10);
	//ros::Duration(5).sleep();


	// int count = device.GetValue(_CR, 1);
	// cout << "Relative Encoder Count: "<< count << endl;


}


void Mtr_Connect::default_config()
{
	////////////////   GENERAL   ///////////////////////
	cout << "- Set Under Voltage Limit: SetConfig(_UVL, "<<50<<")...";  
	if ((status = device.SetConfig(_UVL, 50)) != RQ_SUCCESS)
		cout << "failed --> " << status << endl;
	else
		cout << "succeeded." << endl;
	sleepms(10);

	cout << "- Set Over Voltage Limit: SetConfig(_OVL, "<<400<<")...";  
	if ((status = device.SetConfig(_OVL, 400)) != RQ_SUCCESS)
		cout << "failed --> " << status << endl;
	else
		cout << "succeeded." << endl;
	sleepms(10);

	// cout << "- Set Mixed Mode: SetConfig(_MXMD, "<<1<<")...";  
	// if ((status = device.SetConfig(_MXMD, 1)) != RQ_SUCCESS)
	// 	cout << "failed --> " << status << endl;
	// else
	// 	cout << "succeeded." << endl;
	// sleepms(10);


	////////////////   CHANNEL 1 ///////////////////////
	cout << "- Set Operating Mode: SetConfig(_MMOD, 1, "<<3<<")...";  
	if ((status = device.SetConfig(_MMOD, 1, 3)) != RQ_SUCCESS)
		cout << "failed --> " << status << endl;
	else
		cout << "succeeded." << endl;
	sleepms(10);

	cout << "- Set Encoder as Feedback: SetConfig(_EMOD, 1, "<<18<<")...";  
	if ((status = device.SetConfig(_EMOD, 1, 18)) != RQ_SUCCESS)
		cout << "failed --> " << status << endl;
	else
		cout << "succeeded." << endl;
	sleepms(10);

	cout << "- Set Encoder Lower Limit: SetConfig(_ELL, 1, "<<-5000000<<")...";  
	if ((status = device.SetConfig(_ELL, 1, -5000000)) != RQ_SUCCESS)
		cout << "failed --> " << status << endl;
	else
		cout << "succeeded." << endl;
	sleepms(10);

	cout << "- Set Encoder High Limit: SetConfig(_EHL, 1, "<<5000000<<")...";  
	if ((status = device.SetConfig(_EHL, 1, 5000000)) != RQ_SUCCESS)
		cout << "failed --> " << status << endl;
	else
		cout << "succeeded." << endl;
	sleepms(10);

	cout << "- Set Encoder PPR: SetConfig(_EPPR, 1, "<<ppr<<")...";
	if ((status = device.SetConfig(_EPPR, 1, ppr)) != RQ_SUCCESS)
		cout << "failed --> " << status << endl;
	else
		cout << "succeeded." << endl;
	sleepms(10);

	cout << "- Set Max RPM: SetConfig(_MXRPM, 1, "<<12.0<<")...";
	if ((status = device.SetConfig(_MXRPM, 1, 12)) != RQ_SUCCESS)
		cout << "failed --> " << status << endl;
	else
		cout << "succeeded." << endl;
	sleepms(10);

	cout << "- Set Default Velocity: SetConfig(_MVEL, 1, "<<5.0<<")...";
	if ((status = device.SetConfig(_MVEL, 1, 8)) != RQ_SUCCESS)
		cout << "failed --> " << status << endl;
	else
		cout << "succeeded." << endl;
	sleepms(10);

	cout << "- Set Proportional Gain: SetConfig(_KP, 1, "<<1.0<<")...";
	if ((status = device.SetConfig(_KP, 1, 1000000)) != RQ_SUCCESS)
		cout << "failed --> " << status << endl;
	else
		cout << "succeeded." << endl;
	sleepms(10);

	cout << "- Set Integral Gain: SetConfig(_KI, 1, "<<0<<")...";
	if ((status = device.SetConfig(_KI, 1, 0)) != RQ_SUCCESS)
		cout << "failed --> " << status << endl;
	else
		cout << "succeeded." << endl;
	sleepms(10);

	cout << "- Set Integral Gain: SetConfig(_KD, 1, "<<0<<")...";
	if ((status = device.SetConfig(_KD, 1, 0)) != RQ_SUCCESS)
		cout << "failed --> " << status << endl;
	else
		cout << "succeeded." << endl;
	sleepms(10);

	cout << "- Set Closed Loop Error Detection: SetConfig(_CLERD, 1, "<<0<<")...";  
	if ((status = device.SetConfig(_CLERD, 1, 0)) != RQ_SUCCESS)
		cout << "failed --> " << status << endl;
	else
		cout << "succeeded." << endl;
	sleepms(10);

	cout << "- Set Acceleration: SetCommand(_MAC, 1, "<<100.0<<")..."; 
	if ((status = device.SetCommand(_MAC, 1, 1000)) != RQ_SUCCESS)
		cout << "failed --> " << status << endl;
	else
		cout << "succeeded." << endl;
	sleepms(10);

	cout << "- Set Deceleration: SetCommand(_MDEC, 1, "<<100.0<<")..."; 
	if ((status = device.SetCommand(_MDEC, 1, 1000)) != RQ_SUCCESS)
		cout << "failed --> " << status << endl;
	else
		cout << "succeeded." << endl;
	sleepms(10);

	cout << "- Set Current Threshold: SetConfig(_ATRIG, 1, "<<20.0<<")..."; 
	if ((status = device.SetConfig(_ATRIG, 1, 200)) != RQ_SUCCESS)
		cout << "failed --> " << status << endl;
	else
		cout << "succeeded." << endl;
	sleepms(10);

	cout << "- Set Current Threshold Action (E-stop): SetConfig(_ATGA, 1, "<<2<<")..."; 
	if ((status = device.SetConfig(_ATGA, 1, 2)) != RQ_SUCCESS)
		cout << "failed --> " << status << endl;
	else
		cout << "succeeded." << endl;
	sleepms(10);

	cout << "- Set Left LS Motor 1 (Stop Motor): SetConfig(_DINA, 3, "<<19<<")..."; 
	if ((status = device.SetConfig(_DINA, 3, 19)) != RQ_SUCCESS)
		cout << "failed --> " << status << endl;
	else
		cout << "succeeded." << endl;
	sleepms(10);

	cout << "- Set Right LS Motor 1 (Stop Motor): SetConfig(_DINA, 4, "<<19<<")..."; 
	if ((status = device.SetConfig(_DINA, 4, 19)) != RQ_SUCCESS)
		cout << "failed --> " << status << endl;
	else
		cout << "succeeded." << endl;
	sleepms(10);

	cout << "- Left LS (Reset Home Count Motor 1): SetConfig(_DINA, 3, "<<24<<")..."; 
	if ((status = device.SetConfig(_DINA, 3, 24)) != RQ_SUCCESS)
		cout << "failed --> " << status << endl;
	else
		cout << "succeeded." << endl;
	sleepms(10);


	////////////////   CHANNEL 2 ///////////////////////
	cout << "- Set Operating Mode: SetConfig(_MMOD, 2, "<<3<<")...";  
	if ((status = device.SetConfig(_MMOD, 2, 3)) != RQ_SUCCESS)
		cout << "failed --> " << status << endl;
	else
		cout << "succeeded." << endl;
	sleepms(10);

	cout << "- Set Encoder as Feedback: SetConfig(_EMOD, 2, "<<34<<")...";  
	if ((status = device.SetConfig(_EMOD, 2, 34)) != RQ_SUCCESS)
		cout << "failed --> " << status << endl;
	else
		cout << "succeeded." << endl;
	sleepms(10);

	cout << "- Set Encoder Lower Limit: SetConfig(_ELL, 2, "<<-5000000<<")...";  
	if ((status = device.SetConfig(_ELL, 2, -5000000)) != RQ_SUCCESS)
		cout << "failed --> " << status << endl;
	else
		cout << "succeeded." << endl;
	sleepms(10);

	cout << "- Set Encoder High Limit: SetConfig(_EHL, 2, "<<5000000<<")...";  
	if ((status = device.SetConfig(_EHL, 2, 5000000)) != RQ_SUCCESS)
		cout << "failed --> " << status << endl;
	else
		cout << "succeeded." << endl;
	sleepms(10);

	cout << "- Set Encoder PPR: SetConfig(_EPPR, 2, "<<ppr<<")...";
	if ((status = device.SetConfig(_EPPR, 2, ppr)) != RQ_SUCCESS)
		cout << "failed --> " << status << endl;
	else
		cout << "succeeded." << endl;
	sleepms(10);

	cout << "- Set Max RPM: SetConfig(_MXRPM, 2, "<<12.0<<")...";
	if ((status = device.SetConfig(_MXRPM, 2, 12)) != RQ_SUCCESS)
		cout << "failed --> " << status << endl;
	else
		cout << "succeeded." << endl;
	sleepms(10);

	cout << "- Set Default Velocity: SetConfig(_MVEL, 2, "<<5.0<<")...";
	if ((status = device.SetConfig(_MVEL, 2, 8)) != RQ_SUCCESS)
		cout << "failed --> " << status << endl;
	else
		cout << "succeeded." << endl;
	sleepms(10);

	cout << "- Set Proportional Gain: SetConfig(_KP, 2, "<<1.0<<")...";
	if ((status = device.SetConfig(_KP, 2, 1000000)) != RQ_SUCCESS)
		cout << "failed --> " << status << endl;
	else
		cout << "succeeded." << endl;
	sleepms(10);

	cout << "- Set Integral Gain: SetConfig(_KI, 2, "<<0<<")...";
	if ((status = device.SetConfig(_KI, 2, 0)) != RQ_SUCCESS)
		cout << "failed --> " << status << endl;
	else
		cout << "succeeded." << endl;
	sleepms(10);

	cout << "- Set Integral Gain: SetConfig(_KD, 2, "<<0<<")...";
	if ((status = device.SetConfig(_KD, 2, 0)) != RQ_SUCCESS)
		cout << "failed --> " << status << endl;
	else
		cout << "succeeded." << endl;
	sleepms(10);

	cout << "- Set Closed Loop Error Detection: SetConfig(_CLERD, 2, "<<0<<")...";  
	if ((status = device.SetConfig(_CLERD, 2, 0)) != RQ_SUCCESS)
		cout << "failed --> " << status << endl;
	else
		cout << "succeeded." << endl;
	sleepms(10);

	cout << "- Set Acceleration: SetCommand(_MAC, 2, "<<100.0<<")..."; 
	if ((status = device.SetCommand(_MAC, 2, 1000)) != RQ_SUCCESS)
		cout << "failed --> " << status << endl;
	else
		cout << "succeeded." << endl;
	sleepms(10);

	cout << "- Set Deceleration: SetCommand(_MDEC, 2, "<<100.0<<")..."; 
	if ((status = device.SetCommand(_MDEC, 2, 1000)) != RQ_SUCCESS)
		cout << "failed --> " << status << endl;
	else
		cout << "succeeded." << endl;
	sleepms(10);

	cout << "- Set Current Threshold: SetConfig(_ATRIG, 2, "<<20.0<<")..."; 
	if ((status = device.SetConfig(_ATRIG, 2, 200)) != RQ_SUCCESS)
		cout << "failed --> " << status << endl;
	else
		cout << "succeeded." << endl;
	sleepms(10);

	cout << "- Set Current Threshold Action (E-stop): SetConfig(_ATGA, 2, "<<2<<")..."; 
	if ((status = device.SetConfig(_ATGA, 2, 2)) != RQ_SUCCESS)
		cout << "failed --> " << status << endl;
	else
		cout << "succeeded." << endl;
	sleepms(10);

	cout << "- Set Left LS Motor 2 (Stop Motor): SetConfig(_DINA, 3, "<<35<<")..."; 
	if ((status = device.SetConfig(_DINA, 3, 35)) != RQ_SUCCESS)
		cout << "failed --> " << status << endl;
	else
		cout << "succeeded." << endl;
	sleepms(10);

	cout << "- Set Right LS Motor 2 (Stop Motor): SetConfig(_DINA, 4, "<<35<<")..."; 
	if ((status = device.SetConfig(_DINA, 4, 35)) != RQ_SUCCESS)
		cout << "failed --> " << status << endl;
	else
		cout << "succeeded." << endl;
	sleepms(10);

	cout << "- Left LS (Reset Home Count Motor 2): SetConfig(_DINA, 3, "<<40<<")..."; 
	if ((status = device.SetConfig(_DINA, 3, 40)) != RQ_SUCCESS)
		cout << "failed --> " << status << endl;
	else
		cout << "succeeded." << endl;
	sleepms(10);
}

void Mtr_Connect::cmd_pos_callback(arm_sanding_utilities::SledPosition sp)
{
	float pos = sp.position;
	float circ = 2.0*r_belt*M_PI;
	float num_rev = pos/circ;
	int en_count = int (num_rev*ppr*4.0);  //four counts per pulse
	en_count = 9053;
	cout<< "count: " << en_count <<endl;
	this->move2pos(en_count);
}

void Mtr_Connect::complete_callback(arm_sanding_utilities::PlanCompleteStatus ps)
{
	if (ps.percentcomplete == 100.0)
		plan_complete = true;
}


// Mtr_Connect::calib_callback(arm_sanding_utilities::SledCalibration)
// {

// 	///////////////////////// CHANNEL 1 ///////////////////////////////////
// 	cout << "- Set Default Velocity: SetConfig(_MVEL, 1, "<<2.0<<")...";
// 	if ((status = device.SetConfig(_MVEL, 1, 2)) != RQ_SUCCESS)
// 		cout << "failed --> " << status << endl;
// 	else
// 		cout << "succeeded." << endl;
// 	sleepms(10);

// 	///////////////////////// CHANNEL 2 ///////////////////////////////////
// 	cout << "- Set Default Velocity: SetConfig(_MVEL, 2, "<<2.0<<")...";
// 	if ((status = device.SetConfig(_MVEL, 2, 2)) != RQ_SUCCESS)
// 		cout << "failed --> " << status << endl;
// 	else
// 		cout << "succeeded." << endl;
// 	sleepms(10);

// 	this->move2pos(-4000000);

// }

