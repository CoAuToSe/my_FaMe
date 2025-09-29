import { CheckboxEntry, TextFieldEntry } from '@bpmn-io/properties-panel';

import { getExtensionElement, getServices } from '../../Utils';
//import ExtensionList from './ExtensionList';


export default function ParameterProps(props) {

  const {
    idPrefix,
    parameter,
    element
  } = props;

  const entries = [
    {
      id: idPrefix + '-output',
      component: IsOutput,
      idPrefix,
      parameter
    },
    {
      id: idPrefix + '-name',
      component: Name,
      idPrefix,
      parameter
    }
  ];

  if (!element.businessObject.isOutput) {
    entries.push(
      {
        id: idPrefix + '-value',
        component: Value,
        idPrefix,
        parameter
      }
    )
    // if the data is an output, put the data value to ${data_name}
  } else {
    parameter.value = '${' + parameter.name + '}'
  }

  return entries;
}

function IsOutput(props) {

  const { commandStack, translate, bpmnFactory } = getServices()

  const { element, id } = props;

  let extensionElements = getExtensionElement(element, commandStack, bpmnFactory)

  const setValue = (value) => {
    console.log(value)

    commandStack.execute('element.updateModdleProperties', {
      element,
      moddleElement: extensionElements,
      properties: {
        is_output: value
      }
    });

    element.businessObject.isOutput = value

  };

  const getValue = () => {
    if (!element.businessObject.isOutput)
      element.businessObject.isOutput = false
    return element.businessObject.isOutput;
  };

  return CheckboxEntry({
    element: element,
    id: id + '-output',
    label: translate('is output?'),
    getValue: getValue,
    setValue: setValue
  })
}

function Name(props) {
  const {
    idPrefix,
    element,
    parameter
  } = props;

  const { commandStack, translate, debounce } = getServices()


  const setValue = (value) => {
    commandStack.execute('element.updateModdleProperties', {
      element,
      moddleElement: parameter,
      properties: {
        name: value
      }
    });
  };

  const getValue = (parameter) => {
    return parameter.name;
  };

  return TextFieldEntry({
    element: parameter,
    id: idPrefix + '-name',
    label: translate('Name'),
    getValue,
    setValue,
    debounce
  });
}

function Value(props) {
  const {
    idPrefix,
    element,
    parameter
  } = props;

  const { commandStack, translate, debounce } = getServices()


  const setValue = (value) => {
    commandStack.execute('element.updateModdleProperties', {
      element,
      moddleElement: parameter,
      properties: {
        value: value
      }
    });
  };

  const getValue = (parameter) => {
    return parameter.value;
  };

  return TextFieldEntry({
    element: parameter,
    id: idPrefix + '-value',
    label: translate('Value'),
    getValue,
    setValue,
    debounce
  });
}

