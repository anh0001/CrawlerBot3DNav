var ros;

window.connectRos = function() {
  var url = document.getElementById('websocket_url').value;
  ros = new ROSLIB.Ros({
    url: url
  });

  ros.on('connection', function() {
    console.log('Connected to websocket server.');
    setupTopics();
  });

  ros.on('error', function(error) {
    console.log('Error connecting to websocket server: ', error);
  });

  ros.on('close', function() {
    console.log('Connection to websocket server closed.');
  });
}

function setupTopics() {
  setupCameraTopic();
  setupLidarTopic();
}

window.startRecording = function() {
  var startMsg = new ROSLIB.Message({ data: 'start' });
  var recordingTopic = new ROSLIB.Topic({
    ros: ros,
    name: '/recording_control',
    messageType: 'std_msgs/String'
  });
  recordingTopic.publish(startMsg);
}

window.stopRecording = function() {
  var stopMsg = new ROSLIB.Message({ data: 'stop' });
  var recordingTopic = new ROSLIB.Topic({
    ros: ros,
    name: '/recording_control',
    messageType: 'std_msgs/String'
  });
  recordingTopic.publish(stopMsg);
}

window.pauseRecording = function() {
  var pauseMsg = new ROSLIB.Message({ data: 'pause' });
  var recordingTopic = new ROSLIB.Topic({
    ros: ros,
    name: '/recording_control',
    messageType: 'std_msgs/String'
  });
  recordingTopic.publish(pauseMsg);
}