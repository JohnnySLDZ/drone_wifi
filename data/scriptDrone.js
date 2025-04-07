

let scene, camera, rendered, cube;

function parentWidth(elem) {
  return elem.parentElement.clientWidth;
}

function parentHeight(elem) {
  return elem.parentElement.clientHeight;
}

function init3D(){
  scene = new THREE.Scene();
  scene.background = new THREE.Color(0xffffff);

  camera = new THREE.PerspectiveCamera(75, parentWidth(document.getElementById("3Dcube")) / parentHeight(document.getElementById("3Dcube")), 0.1, 1000);

  renderer = new THREE.WebGLRenderer({ antialias: true });
  renderer.setSize(parentWidth(document.getElementById("3Dcube")), parentHeight(document.getElementById("3Dcube")));

  document.getElementById('3Dcube').appendChild(renderer.domElement);

  // Create a geometry
  const geometry = new THREE.BoxGeometry(5,1,4);
  //BoxGeometry(5,1,4);
  //DodecahedronGeometry(3, 0);

  // Materials of each face
  var cubeMaterials = [
    new THREE.MeshBasicMaterial({color:0x00795e}), //0x03045e
    new THREE.MeshBasicMaterial({color:0x18ba96}),//0x023e8a
    new THREE.MeshBasicMaterial({color:0x43d0b1}),//0x00795e
    new THREE.MeshBasicMaterial({color:0x00795e}),
    new THREE.MeshBasicMaterial({color:0x18ba96}),
    new THREE.MeshBasicMaterial({color:0x43d0b1}),
  ];

  const material = new THREE.MeshFaceMaterial(cubeMaterials);

  cube = new THREE.Mesh(geometry, material);
  scene.add(cube);
  camera.position.z = 5;
  renderer.render(scene, camera);
}

// Resize the 3D object when the browser window changes size
function onWindowResize(){
  camera.aspect = parentWidth(document.getElementById("3Dcube")) / parentHeight(document.getElementById("3Dcube"));
  //camera.aspect = window.innerWidth /  window.innerHeight;
  camera.updateProjectionMatrix();
  //renderer.setSize(window.innerWidth, window.innerHeight);
  renderer.setSize(parentWidth(document.getElementById("3Dcube")), parentHeight(document.getElementById("3Dcube")));

}

window.addEventListener('resize', onWindowResize, false);

// Create the 3D representation
init3D();

// Create events for the sensor readings
if (!!window.EventSource) {
  var source = new EventSource('/events');

  source.addEventListener('open', function(e) {
    console.log("Events Connected");
  }, false);

  source.addEventListener('error', function(e) {
    if (e.target.readyState != EventSource.OPEN) {
      console.log("Events Disconnected");
    }
  }, false);

  source.addEventListener('mpu6050_readings', function(e) {
    //console.log("gyro_readings", e.data);
    var obj = JSON.parse(e.data);
    document.getElementById("yaw").innerHTML = obj.yaw;
    document.getElementById("pitch").innerHTML = obj.pitch;
    document.getElementById("roll").innerHTML = obj.roll;

    // Change cube rotation after receiving the readinds
    cube.rotation.x = obj.roll;
    cube.rotation.z = obj.pitch;
    cube.rotation.y = obj.yaw;
    renderer.render(scene, camera);
  }, false);

  source.addEventListener('bmp280_reading', function(e) {
    //console.log("bmp280_reading", e.data);
    document.getElementById("height").innerHTML = e.data;
  }, false);

  source.addEventListener('battery_reading', function(e) {
    //console.log("battery_reading", e.data);
    document.getElementById("batt").innerHTML = e.data;
  }, false);

  source.addEventListener('throttle_reding', function(e) {
    //console.log("throttle_reding", e.data);
    document.getElementById("throttle_redings").innerHTML = e.data;
  }, false);
}

function get_request(element){
  let method;
  if(element.id == 'throttle'){
    method = '/' + element.id + '?key=' + element.value;
  } else{
    method = '/' + element.id;
  }
  fetch(method).then(response => {
    if (!response.ok) {
      throw new Error('Server Error');
    }
    return response.text();
  })
  .then(data => {
    //console.log('Response:', data);
  })
  .catch(error => {
    console.error('Error:', error);
  });
}
