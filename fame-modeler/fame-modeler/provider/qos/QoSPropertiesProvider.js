// Import your custom property entries.
// The entry is a text input field with logic attached to create,
// update and delete the "spell" property.
import { Group } from '@bpmn-io/properties-panel';

import { isSignalSupported } from '../../utils/EventDefinitionUtil';
import { is, getBusinessObject } from 'bpmn-js/lib/util/ModelUtil';
import QoSProps from './parts/QoSProps';

const LOW_PRIORITY = 500;


/**
 * A provider with a `#getGroups(element)` method
 * that exposes groups for a diagram element.
 *
 * @param {PropertiesPanel} propertiesPanel
 * @param {Function} translate
 */
export default function QoSPropertiesProvider(propertiesPanel, translate) {

  // API ////////

  /**
   * Return the groups provided for the given element.
   *
   * @param {DiagramElement} element
   *
   * @return {(Object[]) => (Object[])} groups middleware
   */
  this.getGroups = function(element) {

    /**
     * We return a middleware that modifies
     * the existing groups.
     *
     * @param {Object[]} groups
     *
     * @return {Object[]} modified groups
     */
    return function(groups) {

      // Add the "qos" group
      if (isSignalSupported(element)) {
        groups.push(createqosGroup(element, translate));
      }

      groups = groups.filter(el => (el.id.includes('Condition') || !el.id.includes('Camunda') || el.id.includes('CallActivity')|| el.id.includes('Script'))); //excludes unused panels

      return groups;
    }
  };


  // registration ////////

  // Register our custom qos properties provider.
  // Use a lower priority to ensure it is loaded after
  // the basic BPMN properties.
  propertiesPanel.registerProvider(LOW_PRIORITY, this);
}

QoSPropertiesProvider.$inject = [ 'propertiesPanel', 'translate' ];

// Create the custom qos group
function createqosGroup(element, translate) {

  // create a group called "QoS properties".
  const qosGroup = {
    id: 'qos',
    label: translate('QoS Properties'),
    component: Group,
    entries: QoSProps(element)
  };

  return qosGroup
}
