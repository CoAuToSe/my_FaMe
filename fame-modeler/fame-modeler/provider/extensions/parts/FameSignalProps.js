import { html } from 'htm/preact';
import { SelectEntry, isSelectEntryEdited, TextFieldEntry, isTextFieldEntryEdited } from '@bpmn-io/properties-panel';
import { useService } from 'bpmn-js-properties-panel';

import { useEffect, useState } from '@bpmn-io/properties-panel/preact/hooks';

import {
  getBusinessObject
} from 'bpmn-js/lib/util/ModelUtil';

import { is } from 'bpmn-js/lib/util/ModelUtil';

import {
  createElement,
  createSignalParameters,
  getSignalParameters,
  createSignalTopic,
  getSignalTopicExtension,
  getSignalParametersExtension,
} from '../Utils';

const backURL = 'http://localhost:9000'; //db connection url

export default function (element) {

  const entries = [{
    id: 'fameSignalType',
    element,
    component: FameSignalType,
    isEdited: isSelectEntryEdited
  }];


  // Check if it is a throwing signal to add payload parameters
  if (is(element, 'bpmn:EndEvent') || is(element, 'bpmn:IntermediateThrowEvent')) {

    console.log('this_state: ', useState())


    const [types, setMessages] = useState([]);

    useEffect(() => {
      function fetchMessages() {
        fetch(backURL + '/messages')
          .then(res => res.json())
          .then(allMsgs => {
            let options = [];
            var diagrams = allMsgs.data;
            diagrams.forEach(element => {
              var idtype = element.class + '/msg/' + element.type
              var payload = element.payload.split(';')
              options.push({ type: idtype, payload: payload })
            });
            setMessages(options);
          })
          .catch(error => console.error(error));
      }

      fetchMessages();
    }, [setMessages]);


    // Add payload fields based on the message type
    var type_payload = types.find(el => el.type == element.businessObject.message_type)

    if (type_payload) {
      type_payload.payload.forEach(tp => {
        entries.push({
          id: tp,
          tp,
          component: FameSignalMessage,
          isEdited: isTextFieldEntryEdited
        });
      });
    }
  }

  return entries;
}

/**
 * Customization of the messages type selection
 * @param {*} props 
 * @returns 
 */
function FameSignalType(props) {
  const { element, id } = props;

  const modeling = useService('modeling');
  const translate = useService('translate');
  const debounce = useService('debounceInput');
  const commandStack = useService('commandStack');
  const bpmnFactory = useService('bpmnFactory');

  const getValue = () => {
    if (!element.businessObject.message_type)
      element.businessObject.message_type = ''
    return element.businessObject.message_type || 0;
  }

  const setValue = value => {
    const businessObject = getBusinessObject(element);
    let extensionElements = businessObject.get('extensionElements');

    // Create new extension element
    if (!extensionElements) {
      extensionElements = createElement(
        'bpmn:ExtensionElements',
        { values: [] },
        businessObject,
        bpmnFactory
      );

      commandStack.execute('element.updateModdleProperties', {
        element,
        moddleElement: businessObject,
        properties: { extensionElements }
      });
    } else {
      // if the ROS message type is changed, clean previous values from the model
      if(extensionElements.values.length > 0)
        extensionElements.values = []
    }

    // Add ROS message type information
    let msg_extension = getSignalTopicExtension(element);

      msg_extension = createSignalTopic({
        type: []
      }, extensionElements, bpmnFactory);


      commandStack.execute('element.updateModdleProperties', {
        element,
        moddleElement: extensionElements,
        properties: {
          values: [...extensionElements.get('values'), msg_extension]
        }
      });

      const newTopic = createElement('ros:message', {
        name: id,
        value: value
      }, msg_extension, bpmnFactory);

      commandStack.execute('element.updateModdleProperties', {
        element,
        moddleElement: msg_extension,
        properties: {
          type: value
        }
      });
    

    element.businessObject.message_type = value
  }

  const [types, setMessages] = useState([]);

  /**
   * Creates the select based on messages type in the db
   */
  useEffect(() => {
    function fetchMessages() {
      fetch(backURL + '/messages')
        .then(res => res.json())
        .then(allMsgs => {
          let options = [];
          var diagrams = allMsgs.data;

          diagrams.forEach(element => {
            var idtype = element.class + '/msg/' + element.type
            options.push(idtype)
          });
          setMessages(options);
        })
        .catch(error => console.error(error));
    }

    fetchMessages();
  }, [setMessages]);


  const getOptions = () => {
    return [
      { label: '<none>', value: undefined },
      ...types.map(type => ({
        label: type,
        value: type
      }))
    ];
  }

  return html`<${SelectEntry}
  element=${element}
  id=${id}
  label=${translate('message_type')}
  getValue=${getValue}
  setValue=${setValue}
  getOptions=${getOptions}
  debounce=${debounce}
  />`
}

function FameSignalMessage(props) {

  const { element, id } = props;

  const commandStack = useService('commandStack');
  const translate = useService('translate');
  const debounce = useService('debounceInput');
  const bpmnFactory = useService('bpmnFactory');

  const setValue = (value) => {

    const businessObject = getBusinessObject(element);

    let extensionElements = businessObject.get('extensionElements');

    if (!extensionElements) {
      extensionElements = createElement(
        'bpmn:ExtensionElements',
        { values: [] },
        businessObject,
        bpmnFactory
      );

      commandStack.execute('element.updateModdleProperties', {
        element,
        moddleElement: businessObject,
        properties: { extensionElements }
      });
    }


    // Message Payload

    let extension = getSignalParametersExtension(element);

    if (!extension) {
      extension = createSignalParameters({
        values: []
      }, extensionElements, bpmnFactory);

      commandStack.execute('element.updateModdleProperties', {
        element,
        moddleElement: extensionElements,
        properties: {
          values: [...extensionElements.get('values'), extension]
        }
      });

    } 

    if (extension.values.length > 0) {
      const parameters = extension.values.filter(value => value.name == id);
      if (parameters.length > 0) {
        commandStack.execute('element.updateModdleProperties', {
          element,
          moddleElement: parameters[0],
          properties: {
            value: value
          }
        });
        return;
      }
    }

    const newParameter = createElement('ros:parameter', {
      name: id,
      value: value
    }, extension, bpmnFactory);

    commandStack.execute('element.updateModdleProperties', {
      element,
      moddleElement: extension,
      properties: {
        values: [...extension.get('values'), newParameter]
      }
    });

  };

  const getValue = () => {
    const businessObject = getBusinessObject(element);
    let extensionElements = businessObject.get('extensionElements');
    if (extensionElements) {
      let extension = getSignalParametersExtension(element);
      if (extension) {
        if (extension.values.length > 0) {
          let values = extension.values.filter(value => value.name == id);
          if (values.length > 0) {
            return values[0].value;
          }
        }
      }
    }
    return '';
  };

  return html`<${TextFieldEntry}
  element=${element}
  id=${id}
  label=${id}
  getValue=${getValue}
  setValue=${setValue}
  debounce=${debounce}
  />`
}