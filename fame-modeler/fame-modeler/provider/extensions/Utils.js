import Ids from 'ids';

import { getBusinessObject } from 'bpmn-js/lib/util/ModelUtil';
import { useService } from 'bpmn-js-properties-panel';


export function getParametersExtension(element) {
  const businessObject = getBusinessObject(element);
  return getExtension(businessObject, 'data:parameters');
}

export function getSignalParametersExtension(element) {
  const businessObject = getBusinessObject(element);
  return getExtension(businessObject, 'ros:payload');
}

export function getSignalTopicExtension(element) {
  const businessObject = getBusinessObject(element);
  return getExtension(businessObject, 'ros:message');
}


export function getSignalQoSExtension(element) {
  const businessObject = getBusinessObject(element);
  return getExtension(businessObject, 'qos:policy');
}

export function getCalledProcessExtension(element) {
  const businessObject = getBusinessObject(element);
  return getExtension(businessObject, 'callActivity:process');
}

export function getParameters(element) {
  const parameters = getParametersExtension(element);
  return parameters && parameters.get('values');
}

export function getSignalParameters(element) {
  const parameters = getSignalParametersExtension(element);
  return parameters && parameters.get('values');
}

export function getExtension(element, type) {
  if (!element.extensionElements) {
    return null;
  }

  return element.extensionElements.values.filter(function (e) {
    return e.$instanceOf(type);
  })[0];
}

export function createElement(elementType, properties, parent, factory) {
  const element = factory.create(elementType, properties);

  if (parent) {
    element.$parent = parent;
  }

  return element;
}

export function createParameters(properties, parent, bpmnFactory) {
  return createElement('data:parameters', properties, parent, bpmnFactory);
}

export function createSignalParameters(properties, parent, bpmnFactory) {
  return createElement('ros:payload', properties, parent, bpmnFactory);
}

export function createSignalTopic(properties, parent, bpmnFactory) {
  return createElement('ros:message', properties, parent, bpmnFactory);
}

export function createSignalQoS(properties, parent, bpmnFactory) {
  return createElement('qos:policy', properties, parent, bpmnFactory);
}

export function createCalledProcess(properties, parent, bpmnFactory) {
  return createElement('callActivity:process', properties, parent, bpmnFactory);
}

export function nextId(prefix) {
  const ids = new Ids([32, 32, 1]);

  return ids.nextPrefixed(prefix);
}

/**
 * Get the list of useful services
 * @returns commandStack, translate, bpmnFactory, debounce
 */
export function getServices() {
  const commandStack = useService('commandStack');
  const translate = useService('translate');
  const bpmnFactory = useService('bpmnFactory');
  const debounce = useService('debounceInput');

  return { commandStack, translate, bpmnFactory, debounce }
}

export function getExtensionElement(element, commandStack, bpmnFactory) {

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
  }
  return extensionElements;
}