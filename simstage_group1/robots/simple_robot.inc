define laser ranger(
	sensor(
		range [ 0.0  5.0 ]
		fov 120.0
		samples 120
	)

	color "blue"
	size [ 0.06 0.06 0.06 ]

	block( points 4
		point[0] [0 0]
		point[1] [0 1]
		point[2] [1 1]
		point[3] [1 0]
		z [0 1]
	) 
  
)

define simple_robot position(
	block
	(
		points 4
		point[0] [0 0]
		point[1] [3 0]
		point[2] [3 3]
		point[3] [0 3]
		z [0 1]
		
	)

	block
	(
		points 4
		point[0] [1 0]
		point[1] [2 0]
		point[2] [2 -0.5]
		point[3] [1 -0.5]
		z [0 0.5]
	)

	block
	(
		points 4
		point[0] [1 3]
		point[1] [2 3]
		point[2] [2 3.5]
		point[3] [1 3.5]
		z [0 0.5]
	)
	color "green"
	odom_error [0.03 0.03 0.00 0.05]
	localization "odom"
	size [0.3 0.3 0.]
	gui_nose 1
	drive "diff"
	laser(pose [ 0.20 0.0 0.2 0.0 ])
)


