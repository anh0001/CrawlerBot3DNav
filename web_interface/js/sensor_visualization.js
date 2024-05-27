function setupCameraTopic() {
    var cameraTopic = new ROSLIB.Topic({
      ros: ros,
      name: '/camera/color/image_raw',
      messageType: 'sensor_msgs/Image'
    });
  
    cameraTopic.subscribe(function(message) {
      var img = document.getElementById('camera_image');
      if (img) {
        img.src = 'data:image/jpeg;base64,' + message.data;
      }
    });
  }
  
  function setupLidarTopic() {
    var lidarTopic = new ROSLIB.Topic({
      ros: ros,
      name: '/lidar/points',
      messageType: 'sensor_msgs/PointCloud2'
    });
  
    lidarTopic.subscribe(function(message) {
      var canvas = document.getElementById('lidar_data');
      if (canvas) {
        var context = canvas.getContext('2d');
        context.clearRect(0, 0, canvas.width, canvas.height);
  
        // Example placeholder for LiDAR data visualization
        context.fillStyle = 'black';
        context.fillText("LiDAR Data", 10, 50);
      }
    });
  }