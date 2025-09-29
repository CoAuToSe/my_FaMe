import 'bpmn-js/dist/assets/diagram-js.css';
import 'bpmn-js/dist/assets/bpmn-js.css';

import 'bpmn-js/dist/assets/bpmn-font/css/bpmn-embedded.css';
import '@bpmn-io/properties-panel/assets/properties-panel.css';

import TokenSimulationModule from '../../lib/viewer';

import BpmnViewer from 'bpmn-js/lib/NavigatedViewer';

import fileDrop from 'file-drops';

import fileOpen from 'file-open';


import { rosInit, rosSubElement, rosPubProcess } from '../utils/ROSconnect';

const url = new URL(window.location.href);

let initialDiagram = sessionStorage.getItem('process');


const viewer = new BpmnViewer({
  container: '#canvas',
  additionalModules: [
    TokenSimulationModule
  ],
  keyboard: {
    bindTo: document
  }
});

function openDiagram(diagram) {
  return viewer.importXML(diagram)
    .then(({ warnings }) => {
      if (warnings.length) {
        console.warn('Uploaded', warnings);
      }
      viewer.get('canvas').zoom('fit-viewport');
    })
    .catch(err => {
      console.error(err);
    });
}

openDiagram(initialDiagram);
/// FaMe Connection ///

const fame = viewer.get("fameConnector");

rosInit()

var sub_topic = rosSubElement()

// topic subscription
sub_topic.subscribe((message) => {
  var res_message = {}
  var response = message.data
  var msg = response.split('/')

  res_message.id = msg[0]
  res_message.status = msg[1]
  res_message.instance = msg[2]

  console.log(res_message)
  switch (res_message.status) {
    case 'start': {
      fame.animateTask(res_message.id, res_message.instance);

      // Appends the activated element in the log list
      var log_base = document.getElementsByClassName('bts-log')[0]
      var log_list = log_base.getElementsByClassName('bts-content')[0]
      var placeholder = log_list.getElementsByClassName('placeholder')[0]
      //log_list.removeChild(placeholder) //TODO fix

      var element_log = document.createElement('div');
      element_log.className = 'bts-entry'
      const new_log = document.createElement("p");
      const text_log = document.createTextNode(res_message.id);
      new_log.appendChild(text_log);
      element_log.appendChild(new_log)
      log_list.appendChild(element_log)

      break;
    }
    case 'stop': {
      fame.deanimateTask(res_message.id)
      break;
    }
    default: {
      fame.animateSequenceFlow(res_message.id, res_message.instance);
      break;
    }
  }
  
});

document.getElementById('send-button').addEventListener('click', function(event){
  rosPubProcess(viewer);
})