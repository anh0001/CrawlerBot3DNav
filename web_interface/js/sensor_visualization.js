function setupCameraTopic() {
  console.log('Setting up camera topic...');
  var cameraTopic = new ROSLIB.Topic({
    ros: ros,
    name: '/camera/color/image_raw',
    messageType: 'sensor_msgs/Image'
  });

  cameraTopic.subscribe(function(message) {
    try {
      // console.log('Received camera image message:', message);

      var width = message.width;
      var height = message.height;
      var data = atob(message.data); // Decode base64 to binary

      // Create a new canvas element
      var canvas = document.createElement('canvas');
      canvas.width = width;
      canvas.height = height;
      var context = canvas.getContext('2d');

      // Create an ImageData object
      var imageData = context.createImageData(width, height);

      // Copy the binary data into the ImageData object
      var i, j;
      for (i = 0, j = 0; i < data.length; i += 3, j += 4) {
        imageData.data[j] = data.charCodeAt(i);      // R
        imageData.data[j + 1] = data.charCodeAt(i + 1);  // G
        imageData.data[j + 2] = data.charCodeAt(i + 2);  // B
        imageData.data[j + 3] = 255;                // A (Alpha)
      }

      // Put the image data to the canvas context
      context.putImageData(imageData, 0, 0);

      // Convert the canvas content to a base64 PNG image
      var img = document.getElementById('camera_image');
      if (img) {
        img.src = canvas.toDataURL('image/png');
      } else {
        console.error('Camera image element not found');
      }
    } catch (e) {
      console.error('Error processing camera image message:', e);
    }
  });
}

function setupLidarTopic() {
  var lidarTopic = new ROSLIB.Topic({
    ros: ros,
    name: '/lidar/points',
    messageType: 'sensor_msgs/PointCloud2'
  });

  lidarTopic.subscribe(function (message) {
    try {
      console.log('Received LiDAR data message:', message);
      var canvas = document.getElementById('lidar_data');
      if (canvas) {
        var context = canvas.getContext('2d');
        context.clearRect(0, 0, canvas.width, canvas.height);

        // Example placeholder for LiDAR data visualization
        context.fillStyle = 'black';
        context.fillText("LiDAR Data", 10, 50);
      } else {
        console.error('LiDAR canvas element not found');
      }
    } catch (e) {
      console.error('Error processing LiDAR data message:', e);
    }
  });
}
