// Function to establish connection to ROS
var ros = new ROSLIB.Ros({
    url: 'ws://localhost:9090'
  });
  
  ros.on('connection', function() {
    console.log('Connected to websocket server.');
    notifyUser('Connected to websocket server.');
  });
  
  ros.on('error', function(error) {
    console.log('Error connecting to websocket server: ', error);
    notifyUser('Error connecting to websocket server: ' + error);
  });
  
  ros.on('close', function() {
    console.log('Connection to websocket server closed.');
    notifyUser('Connection to websocket server closed.');
  });
  
  // Function to display notifications to the user
  function notifyUser(message) {
    document.getElementById('notifications').innerHTML = message;
  }
  
  // Define a ROS topic for recording control
  var recordingTopic = new ROSLIB.Topic({
    ros: ros,
    name: '/recording_control',
    messageType: 'std_msgs/String'
  });
  
  // Functions to control recording
  function startRecording() {
    var startMsg = new ROSLIB.Message({ data: 'start' });
    recordingTopic.publish(startMsg);
    notifyUser('Started recording.');
  }
  
  function stopRecording() {
    var stopMsg = new ROSLIB.Message({ data: 'stop' });
    recordingTopic.publish(stopMsg);
    notifyUser('Stopped recording.');
  }
  
  function pauseRecording() {
    var pauseMsg = new ROSLIB.Message({ data: 'pause' });
    recordingTopic.publish(pauseMsg);
    notifyUser('Paused recording.');
  }
  
  // Subscribe to the camera topic and display images
  var cameraTopic = new ROSLIB.Topic({
    ros: ros,
    name: '/camera/color/image_raw',
    messageType: 'sensor_msgs/Image'
  });
  
  cameraTopic.subscribe(function(message) {
    var img = document.createElement('img');
    img.src = 'data:image/jpeg;base64,' + message.data;
    var cameraFeed = document.getElementById('camera_feed');
    cameraFeed.innerHTML = '';
    cameraFeed.appendChild(img);
  });
  
  // Subscribe to the LiDAR topic and visualize point clouds
  var lidarTopic = new ROSLIB.Topic({
    ros: ros,
    name: '/lidar/points',
    messageType: 'sensor_msgs/PointCloud2'
  });
  
  lidarTopic.subscribe(function(message) {
    var canvas = document.getElementById('lidar_data');
    var context = canvas.getContext('2d');
    context.clearRect(0, 0, canvas.width, canvas.height);
  
    // Process and visualize LiDAR data
    // (Example placeholder for LiDAR data visualization)
    context.fillStyle = 'black';
    context.fillText("LiDAR Data", 10, 50);
  });