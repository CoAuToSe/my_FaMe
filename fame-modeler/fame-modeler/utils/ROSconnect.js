import ROSLIB from 'roslib';

var ros = '';


export function rosInit() {
  ros = new ROSLIB.Ros({ url: "ws://localhost:9090" });

  ros.on("connection", () => {
    console.log('Connected with ROS')
  });

  ros.on("error", (error) => {
    console.log('Error:')
    console.log(error)
  });

  ros.on("close", () => {
    console.log('Closed ROS connection')
  });
}

export function rosConnect(document) {
  // Init ROS connection
  document.querySelector('#ros-button').addEventListener('click', function (event) {
    rosInit()

    ros.on("connection", () => {
      document.getElementById("ros-button").style.background = '#104dd0';
      document.getElementById("ros-button").style.color = '#FFFFFF';
      document.getElementById("ros-button").innerHTML = '<i class="bi bi-robot"></i> Connected with ROS'

      document.getElementById("send-button").style.display = "inline"
    });

    ros.on("close", () => {
      alert('Closed ROS connection')
      document.getElementById("ros-button").style.background = "";
      document.getElementById("ros-button").style.color = "";
      document.getElementById("ros-button").innerHTML = 'Connect with ROS'

      document.getElementById("send-button").style.display = "none"

    });

  });


}

export function rosSubElement() {
  const dt_topic = new ROSLIB.Topic({
    ros,
    name: "/fame_dt",
    messageType: "std_msgs/String",
  });
  return dt_topic
}


export function rosPubProcess(modeler) {
  // Export bpmn as string
  async function getXML() {
    try {
      const xml = await modeler.saveXML({ format: false });
      return xml;
    } catch (err) {
      console.log(err);
    }
  }

  // Handle bpmn publishing over the /collaboration_diagram topic
  document.querySelector('#send-button').addEventListener('click', function (event) {
    const process_topic = new ROSLIB.Topic({
      ros,
      name: "/collaboration_diagram",
      messageType: "std_msgs/String",
    });


    getXML().then(value => {
      var out = value.xml

      var process_message = new ROSLIB.Message({
        data: out
      });

      process_topic.publish(process_message)

      alert('Process published')
      console.log('Process published')

      sessionStorage.setItem('process', out);

      window.open('viewer.html');      
    });
  })
}