import {isAny} from 'bpmn-js/lib/features/modeling/util/ModelingUtil';

import {getBusinessObject, is} from 'bpmn-js/lib/util/ModelUtil';

import {find} from 'min-dash';

/**
 * Teste si `element` correspond au type de `types`,
 * en acceptant "bpmn:Foo" ou "Foo".
 * @param {any} element
 * @param {string} type
 * @param {string} [defaultNs='bpmn']
 */
export function isCompat(element, type, defaultNs = 'bpmn') {
    // enlève n'importe quel préfixe (bpmn:, camunda:, etc.)
    const bare = type.replace(/^[^:]*:/, '');
    const candidates = [`${defaultNs}:${bare}`, bare];
    // dédoublonne au cas où
    const unique = Array.from(new Set(candidates));
    return isAny(element, unique);
}


/**
 * Teste si `element` correspond à au moins un type de `types`,
 * en acceptant "bpmn:Foo" ou "Foo".
 * @param {any} element
 * @param {string|string[]} types
 * @param {string} [defaultNs='bpmn']
 */
export function isCompatAny(element, types, defaultNs = 'bpmn') {
    const arr = Array.isArray(types) ? types : [types];
    const expanded = [];

    for (const t of arr) {
        if (typeof t !== 'string' || !t.trim()) continue;
        const bare = t.replace(/^[^:]*:/, ''); // retire un éventuel préfixe
        expanded.push(`${defaultNs}:${bare}`, bare);
    }

    // dédoublonnage en conservant l’ordre
    const seen = new Set();
    const unique = expanded.filter(v => v && !seen.has(v) && seen.add(v));

    if (unique.length === 0) return false;
    return isAny(element, unique);
}

/** Si tu veux réutiliser l’expansion ailleurs */
export function expandTypes(types, defaultNs = 'bpmn') {
    const arr = Array.isArray(types) ? types : [types];
    const out = [];
    for (const t of arr) {
        if (typeof t !== 'string' || !t.trim()) continue;
        const bare = t.replace(/^[^:]*:/, '');
        out.push(`${defaultNs}:${bare}`, bare);
    }
    const seen = new Set();
    return out.filter(v => v && !seen.has(v) && seen.add(v));
}

export function isErrorSupported(element) {
    return isCompatAny(element, [
        'bpmn:StartEvent',
        'bpmn:BoundaryEvent',
        'bpmn:EndEvent'
    ]) && !!getErrorEventDefinition(element);
}

export function getErrorEventDefinition(element) {
    return getEventDefinition(element, 'bpmn:ErrorEventDefinition');
}

export function isTimerSupported(element) {
    return isCompatAny(element, [
        'bpmn:StartEvent',
        'bpmn:IntermediateCatchEvent',
        'bpmn:BoundaryEvent',
        'StartEvent'
    ]) && !!getTimerEventDefinition(element);
}

/**
 * Get the timer definition type for a given timer event definition.
 *
 * @param {ModdleElement<bpmn:TimerEventDefinition>} timer
 *
 * @return {string|undefined} the timer definition type
 */
export function getTimerDefinitionType(timer) {

    if (!timer) {
        return;
    }

    const timeDate = timer.get('timeDate');
    if (typeof timeDate !== 'undefined') {
        return 'timeDate';
    }

    const timeCycle = timer.get('timeCycle');
    if (typeof timeCycle !== 'undefined') {
        return 'timeCycle';
    }

    const timeDuration = timer.get('timeDuration');
    if (typeof timeDuration !== 'undefined') {
        return 'timeDuration';
    }
}

export function getTimerEventDefinition(element) {
    return getEventDefinition(element, 'bpmn:TimerEventDefinition');
}

export function getError(element) {
    const errorEventDefinition = getErrorEventDefinition(element);

    return errorEventDefinition && errorEventDefinition.get('errorRef');
}

export function getEventDefinition(element, eventType) {
    const businessObject = getBusinessObject(element);

    const eventDefinitions = businessObject.get('eventDefinitions') || [];

    return find(eventDefinitions, function(definition) {
        return isCompat(definition, eventType);
    });
}

export function isMessageSupported(element) {
    return isCompat(element, 'bpmn:ReceiveTask') || (
        isCompatAny(element, [
            'bpmn:StartEvent',
            'bpmn:EndEvent',
            'bpmn:IntermediateThrowEvent',
            'bpmn:BoundaryEvent',
            'bpmn:IntermediateCatchEvent'
        ]) && !!getMessageEventDefinition(element)
    );
}

export function getParametersExtension(element) {
    const businessObject = getBusinessObject(element);
    return getExtension(businessObject, 'data:parameters');
}

export function getParameters(element) {
    const parameters = getParametersExtension(element);
    return parameters && parameters.get('values');
}

export function getExtension(element, type) {
    if (!element.extensionElements) {
        return null;
    }

    return element.extensionElements.values.filter(function(e) {
        return e.$instanceOf(type);
    })[0];
}

export function getMessageEventDefinition(element) {
    if (isCompat(element, 'bpmn:ReceiveTask')) {
        return getBusinessObject(element);
    }

    return getEventDefinition(element, 'bpmn:MessageEventDefinition');
}

export function getMessage(element) {
    const messageEventDefinition = getMessageEventDefinition(element);

    return messageEventDefinition && messageEventDefinition.get('messageRef');
}

export function getLinkEventDefinition(element) {
    return getEventDefinition(element, 'bpmn:LinkEventDefinition');
}

export function getSignalEventDefinition(element) {
    return getEventDefinition(element, 'bpmn:SignalEventDefinition');
}

export function isLinkSupported(element) {
    return isCompatAny(element, [
        'bpmn:IntermediateThrowEvent',
        'bpmn:IntermediateCatchEvent'
    ]) && !!getLinkEventDefinition(element);
}

export function isSignalSupported(element) {
    return isCompat(element, 'bpmn:Event') && !!getSignalEventDefinition(element);
}

export function getSignal(element) {
    const signalEventDefinition = getSignalEventDefinition(element);

    return signalEventDefinition && signalEventDefinition.get('signalRef');
}

export function getEscalationEventDefinition(element) {
    return getEventDefinition(element, 'bpmn:EscalationEventDefinition');
}

export function isEscalationSupported(element) {
    return isCompat(element, 'bpmn:Event') && !!getEscalationEventDefinition(element);
}

export function getEscalation(element) {
    const escalationEventDefinition = getEscalationEventDefinition(element);

    return escalationEventDefinition && escalationEventDefinition.get('escalationRef');
}

export function isCompensationSupported(element) {
    return isCompatAny(element, [
        'bpmn:EndEvent',
        'bpmn:IntermediateThrowEvent'
    ]) && !!getCompensateEventDefinition(element);
}

export function getCompensateEventDefinition(element) {
    return getEventDefinition(element, 'bpmn:CompensateEventDefinition');
}

export function getCompensateActivity(element) {
    const compensateEventDefinition = getCompensateEventDefinition(element);

    return compensateEventDefinition && compensateEventDefinition.get('activityRef');
}
