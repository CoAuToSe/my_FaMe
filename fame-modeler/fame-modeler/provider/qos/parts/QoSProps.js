import { html } from 'htm/preact';
import { SelectEntry, NumberFieldEntry } from '@bpmn-io/properties-panel';
import { isSelectEntryEdited, isNumberFieldEntryEdited } from '@bpmn-io/properties-panel';

import { useService } from 'bpmn-js-properties-panel';

import { getBusinessObject } from 'bpmn-js/lib/util/ModelUtil';

import {
  createElement,
  createSignalQoS,
  getSignalQoSExtension
} from '../../extensions/Utils';

export default function QoSProps(element) {

  const entries = [
    {
      id: 'history',
      element,
      component: History,
      isEdited: isSelectEntryEdited
    },
    {
      id: 'depth',
      element,
      component: Depth,
      isEdited: isNumberFieldEntryEdited
    },
    {
      id: 'reliability',
      element,
      component: Reliability,
      isEdited: isSelectEntryEdited
    },
    {
      id: 'durability',
      element,
      component: Durability,
      isEdited: isSelectEntryEdited
    },
    {
      id: 'deadline',
      element,
      component: Deadline,
      isEdited: isNumberFieldEntryEdited
    },
    {
      id: 'lifespan',
      element,
      component: Lifespan,
      isEdited: isNumberFieldEntryEdited
    },
    {
      id: 'liveliness',
      element,
      component: Liveliness,
      isEdited: isSelectEntryEdited
    },
    {
      id: 'lease_duration',
      element,
      component: LeaseDuration,
      isEdited: isNumberFieldEntryEdited
    }
  ];

  return entries;
}


function History(props) {
  var history = 'history'
  var entries = [
    { label: "Keep Last", value: 0 },
    { label: "Keep All", value: 1 }
    //{ label: "Keep Last", value: "keep_last" },
    //{ label: "Keep All", value: "keep_all" }
  ];
  return Extension(props, history, SelectEntry, entries)
}


function Depth(props) {
  var depth = 'depth'
  return Extension(props, depth, NumberFieldEntry)
}

function Reliability(props) {
  var reliability = 'reliability'
  var entries = [{ label: "Best Effort", value: 0 }, { label: "Reliable", value: 1 }]
  return Extension(props, reliability, SelectEntry, entries)
}

function Durability(props) {
  var durability = 'durability'
  var entries = [{ label: "Transient Local", value: 0 }, { label: "Volatile", value: 1 }];

  return Extension(props, durability, SelectEntry, entries)
}

function Deadline(props) {
  var deadline = 'deadline'
  return Extension(props, deadline, NumberFieldEntry)
}

function Lifespan(props) {
  var lifespan = 'lifespan';
  return Extension(props, lifespan, NumberFieldEntry)
}

function Liveliness(props) {
  var liveliness = 'liveliness'
  var entries = [{ label: "Automatic", value: 0 }, { label: "Manual by Topic", value: 1 }];

  return Extension(props, liveliness, SelectEntry, entries)
}


function LeaseDuration(props) {
  var lease_duration = 'leaseDuration'
  return Extension(props, lease_duration, NumberFieldEntry)
}


/**
 * General function to create QoS property panel
 * @param {*} props 
 * @param {*} policy QoS policy identifier
 * @param {*} out_type Visualization type
 * @param {*} entries Select option choices, default null
 * @returns 
 */
function Extension(props, policy, out_type, entries = undefined) {
  const { element, id } = props;

  const modeling = useService('modeling');
  const translate = useService('translate');
  const debounce = useService('debounceInput');
  const bpmnFactory = useService('bpmnFactory');
  const commandStack = useService('commandStack');

  const getValue = () => {
    console.log(element.businessObject[policy])
    if (!element.businessObject[policy])
      element.businessObject[policy] = 0;
    return element.businessObject[policy] || 0;
  }

  const setValue = value => {
    const businessObject = getBusinessObject(element);

    let extensionElements = businessObject.get('extensionElements');

    console.log('extensionElements')
    console.log(extensionElements)

    /**
     * Create an extension element if not present
     */
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

    let extension = getSignalQoSExtension(element);

    //delete old parameter
    if (extension && extension[policy]) {
      delete extensionElements.values.extension
      extensionElements.values = extensionElements.values.filter(el => !el[policy]);
    }

    var qosextension = createSignalQoS({
      [policy]: value
    }, extensionElements, bpmnFactory);

    commandStack.execute('element.updateModdleProperties', {
      element,
      moddleElement: extensionElements,
      properties: {
        values: [...extensionElements.get('values'), qosextension]
      }
    });

    const newParameter = createElement('qos:policy', {
      name: id,
      value: value,
    }, qosextension, bpmnFactory);

    element.businessObject[id] = value


    commandStack.execute('element.updateModdleProperties', {
      element,
      moddleElement: qosextension,
      properties: {
        type: id, // add the policy id in the type field
        value: value
      }
    });
  }

  if (entries) {
    const getOptions = () => {
      return entries;
    }
    
    return html`<${out_type}
    id=${id}
    element=${element}
    label=${translate(policy)}
    getValue=${getValue}
    setValue=${setValue}
    getOptions=${getOptions}
    debounce=${debounce}
  />`
  }
  else {
    return html`<${out_type}
      id=${id}
      element=${element}
      label=${translate(policy)}
      getValue=${getValue}
      setValue=${setValue}
      debounce=${debounce}
    />`}
}
