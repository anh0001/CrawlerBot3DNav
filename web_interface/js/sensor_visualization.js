function setupCameraTopic() {
  console.log('Setting up camera topic...');
  var cameraTopic = new ROSLIB.Topic({
    ros: ros,
    name: '/camera/color/image_raw',
    messageType: 'sensor_msgs/Image'
  });

  cameraTopic.subscribe(function (message) {
    try {
      // console.log('Received camera image message:', message);

      // Get the image dimensions and data
      var width = message.width;
      var height = message.height;
      var imageData = message.data;

      // Create a new canvas element
      var canvas = document.createElement('canvas');
      canvas.width = width;
      canvas.height = height;

      // Get the canvas context and create an ImageData object
      var context = canvas.getContext('2d');
      var imageDataObject = context.createImageData(width, height);

      // Decode the base64 string to raw binary data
      var binary = atob(imageData);

      // Convert the binary data to Uint8ClampedArray and set it to imageDataObject
      for (var i = 0; i < imageDataObject.data.length; i++) {
        imageDataObject.data[i] = binary.charCodeAt(i);
      }

      // Put the image data to the canvas context
      context.putImageData(imageDataObject, 0, 0);

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
