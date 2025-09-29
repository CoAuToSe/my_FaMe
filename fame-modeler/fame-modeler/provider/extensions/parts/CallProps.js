import { html } from 'htm/preact';
import { HeaderButton, SelectEntry, isSelectEntryEdited, isTextFieldEntryEdited } from '@bpmn-io/properties-panel';
import { useService } from 'bpmn-js-properties-panel';

import { useEffect, useState } from '@bpmn-io/properties-panel/preact/hooks';

import { XMLParser, XMLBuilder } from "fast-xml-parser";
import { getBusinessObject } from 'bpmn-js/lib/util/ModelUtil';
import { createCalledProcess, createElement, getCalledProcessExtension } from '../Utils';

export default function (element) {

  const entries = [
    {
      id: 'calledElement',
      element,
      component: Call,
      isEdited: isSelectEntryEdited
    },
    {
      id: 'newCallActivity',
      element,
      component: CreateNewCall,
      isEdited: isTextFieldEntryEdited,
    }
  ]

  console.log('this' , entries)
  return entries;
}

function Call(props) {
  const { element, id } = props;

  const modeling = useService('modeling');
  const translate = useService('translate');
  const debounce = useService('debounceInput');
  const bpmnFactory = useService('bpmnFactory');
  const commandStack = useService('commandStack');


  console.log(element)



  const backURL = 'http://localhost:9000';


  const getValue = () => {
    return element.businessObject.selection || '';
  }

  const setValue = value => {

    const businessObject = getBusinessObject(element);
    let extensionElements = businessObject.get('extensionElements');

    element.businessObject.selection = value // for property panel rendering

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
    }


    var name = value.split(' :: ')[0]
    var process = value.split(' :: ')[1]
    // Handle call activity process
    process = process.substring(2, process.length - 1)
    //process = process.replaceAll('bpmn2', 'bpmn')
    // Call activity extraction
    const options = {
      ignoreAttributes: false,
      attributeNamePrefix: "@_"
    };
    const parser = new XMLParser(options);
    let obj = parser.parse(process)['bpmn2:definitions'];

    const builder = new XMLBuilder(options);
    var callActivity = builder.build(obj);

    console.log(callActivity)

    console.log(name)

    let process_extension = getCalledProcessExtension(element);
    if (!process_extension) {

      process_extension = createCalledProcess({
        process: callActivity
      }, extensionElements, bpmnFactory);

      commandStack.execute('element.updateModdleProperties', {
        element,
        moddleElement: extensionElements,
        properties: {
          values: [...extensionElements.get('values'), process_extension]
        }
      });
    }

    return modeling.updateProperties(element, {
      calledElement: name,
      name: name
    });
  }

  const [calls, setCalls] = useState([]);

  useEffect(() => {
    function fetchCalls() {
      fetch(backURL + '/processes')
        .then(res => res.json())
        .then(allCalls => {
          let options = [];
          var diagrams = allCalls.data;
          diagrams.forEach(element => {
            options.push({
              name: element.name.trim(),
              process: element.process
            })
          });
          console.log(options)

          setCalls(options);
        })
        .catch(error => console.error(error));
    }
    fetchCalls();
  }, [setCalls]);


  const getOptions = () => {
    return [
      { label: '<none>', value: undefined },
      ...calls.map(call => ({
        label: call.name,
        value: call.name + ' :: ' + call.process,
      }))

    ];
  }

  return html`<${SelectEntry}
  element=${element}
  id=${id}
  label=${translate('Called element')}
  getValue=${getValue}
  setValue=${setValue}
  getOptions=${getOptions}
  debounce=${debounce}
  />`
}

function CreateNewCall(props) {
  const translate = useService('translate');

  const onNew = () => {
    window.open('call_activity_modeler.html')
  }

  return  HeaderButton({
    id : 'add-button',
    class: "button-ca",
    onClick: onNew,
    description: translate("New Element"),
    children: translate("New Element"),
    title: translate("New Element"),
    children: 'New call activity',
  })  
}