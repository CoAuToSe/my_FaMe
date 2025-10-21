/* global process */

import 'bpmn-js/dist/assets/diagram-js.css';
import 'bpmn-js/dist/assets/bpmn-js.css';

import 'bpmn-js/dist/assets/bpmn-font/css/bpmn-embedded.css';
import '@bpmn-io/properties-panel/assets/properties-panel.css';

import TokenSimulationModule from '../..';

import BpmnModeler from 'bpmn-js/lib/Modeler';

import AddExporter from '@bpmn-io/add-exporter';

import qosPropertiesProviderModule from '../provider/qos';
import qosModdleDescriptor from '../descriptors/qos';

import customElementsProvider from '../provider/extensions';
import callModdleDescriptor from '../descriptors/callActivity';
import dataMoodleDescriptor from '../descriptors/dataObject';
import signalDataMoodleDescriptor from '../descriptors/signalData';


import {
    BpmnPropertiesPanelModule,
    BpmnPropertiesProviderModule,
    CamundaPlatformPropertiesProviderModule
} from 'bpmn-js-properties-panel';

import fileDrop from 'file-drops';

import fileOpen from 'file-open';

import download from 'downloadjs';

import exampleXML from '../resources/rexdingo.bpmn';

import camundaModdleDescriptors from 'camunda-bpmn-moddle/resources/camunda.json';

import { pubOrSub, qosCompatibilityCheck } from '../utils/QoSCheck';
import { rosConnect, rosPubProcess } from '../utils/ROSconnect';
import { storeCallAct } from '../utils/CallActivityStore';

const url = new URL(window.location.href);

const persistent = url.searchParams.has('p');
const active = url.searchParams.has('e');
const presentationMode = url.searchParams.has('pm');

let fileName = 'diagram.bpmn';

const initialDiagram = (() => {
    try {
        return persistent && localStorage['diagram-xml'] || exampleXML;
    } catch (err) {
        return exampleXML;
    }
})();

function showMessage(cls, message) {
    const messageEl = document.querySelector('.drop-message');

    messageEl.textContent = message;
    messageEl.className = `drop-message ${cls || ''}`;

    messageEl.style.display = 'block';
}

function hideMessage() {
    const messageEl = document.querySelector('.drop-message');

    messageEl.style.display = 'none';
}

if (persistent) {
    hideMessage();
}
/*
const InitModule = {
    __init__: [
        ['eventBus', 'bpmnjs', 'toggleMode', function (eventBus, bpmnjs, toggleMode) {

            if (persistent) {
                eventBus.on('commandStack.changed', function () {
                    bpmnjs.saveXML().then(result => {
                        localStorage['diagram-xml'] = result.xml;
                    });
                });
            }

            if ('history' in window) {
                eventBus.on('tokenSimulation.toggleMode', event => {

                    document.body.classList.toggle('token-simulation-active', event.active);

                    if (event.active) {
                        url.searchParams.set('e', '1');
                    } else {
                        url.searchParams.delete('e');
                    }

                    history.replaceState({}, document.title, url.toString());
                });
            }

            eventBus.on('diagram.init', 500, () => {
                toggleMode.toggleMode(active);
            });
        }]
    ]
};
*/

const modeler = new BpmnModeler({
    container: '#canvas',
    additionalModules: [
        BpmnPropertiesPanelModule,
        BpmnPropertiesProviderModule,
        CamundaPlatformPropertiesProviderModule,
        qosPropertiesProviderModule,
        customElementsProvider,
        //TokenSimulationModule,
        AddExporter,
        //InitModule
    ],
    propertiesPanel: {
        parent: '#properties-panel'
    },
    exporter: {
        name: 'bpmn-js-token-simulation',
        version: process.env.TOKEN_SIMULATION_VERSION
    },
    keyboard: {
        bindTo: document
    },
    moddleExtensions: {
        camunda: camundaModdleDescriptors,
        qos: qosModdleDescriptor,
        callActivity: callModdleDescriptor,
        dataObject: dataMoodleDescriptor,
        signalData: signalDataMoodleDescriptor
    }
});

function openDiagram(diagram) {
    return modeler.importXML(diagram)
        .then(({ warnings }) => {
            if (warnings.length) {
                console.warn(warnings);
            }

            if (persistent) {
                localStorage['diagram-xml'] = diagram;
            }

            modeler.get('canvas').zoom('fit-viewport');
        })
        .catch(err => {
            console.error(err);
        });
}

if (presentationMode) {
    document.body.classList.add('presentation-mode');
}

function openFile(files) {
    if (!files.length) {
        return;
    }
    //hideMessage();

    fileName = files[0].name;
    console.log(fileName)


    openDiagram(files[0].contents);
}

document.body.addEventListener('dragover', fileDrop('Open BPMN diagram', openFile), false);


async function downloadDiagram() {
    try {
        scriptControl()
        const result = await modeler.saveXML({ format: true });
        const { xml } = result;
        download(xml, fileName, 'application/xml');
        console.log('Model saved!');
    } catch (err) {
        console.log(err);
    }
    /*    modeler.saveXML({ format: true }, function(err, xml) {
            if (!err) {
                download(xml, fileName, 'application/xml');
            }
        });*/
}

async function downloadSVG() {
    try {
        const result = await modeler.saveSVG({ format: true });
        const { svg } = result;
        download(svg, fileName, 'image/svg+xml');
        console.log('Model saved!');
    } catch (err) {
        console.log(err);
    }
}

function scriptControl() {
    const elementRegistry = modeler.get('elementRegistry');
    elementRegistry.forEach(function (element) {
        if ((element.type && element.type == 'bpmn:ScriptTask' || element.type && element.type == 'ScriptTask') && element.businessObject.scriptFormat == 'JavaScript' && element.businessObject.script !== undefined) {
            const oldScript = element.businessObject.script;
            if (!oldScript.includes('next()'))
                element.businessObject.script = oldScript + '\nnext();';
        }
    })
}


const propertiesPanel = document.querySelector('#properties-panel');

const propertiesPanelResizer = document.querySelector('#properties-panel-resizer');

let startX, ì, stopX;

function toggleProperties(open) {
    propertiesPanel.classList.toggle('open', open);
}

propertiesPanelResizer.addEventListener('dblclick', function (event) {
    toggleProperties(!propertiesPanel.classList.contains('open'));
});

propertiesPanelResizer.addEventListener('dragstart', function (event) {
    startX = event.screenX;
});

//#TODO: improvements needed
propertiesPanelResizer.addEventListener('dragend', function (event) {
    stopX = event.screenX;
    var width = 0;
    // if (startX < stopX) {
    //     // width = stopX - startX;
    //     width = stopX;
    //     console.log(startX, stopX);
    // }
    // else {
    //     // width = startX - stopX;
    //     width = stopX;
    //     console.log(startX, stopX);
    // }
    
    width = (window.screen.availWidth + window.innerWidth)/2 - stopX;
    console.log(startX, stopX);
    // propertiesPanel.style.width = open ? `${width}px` : null;
    propertiesPanel.style.width = `${width}px`;

    toggleProperties(open);
});

// Init the connection with ROS
if (document.getElementById('ros-button')) {
    rosConnect(document);
    rosPubProcess(modeler);
}

document.body.addEventListener('keydown', function (event) {
    if (event.code === 'KeyS' && (event.metaKey || event.ctrlKey)) {
        event.preventDefault();

        downloadDiagram();
    }

    if (event.code === 'KeyO' && (event.metaKey || event.ctrlKey)) {
        event.preventDefault();

        fileOpen().then(openFile);
    }
});

document.querySelector('#download-button').addEventListener('click', function (event) {
    downloadDiagram();
});

document.querySelector('#svg-button').addEventListener('click', function (event) {
    downloadSVG();
});

document.querySelector('#new-diagram').addEventListener('click', () => {
    fileOpen().then(openFile);
});

const remoteDiagram = url.searchParams.get('diagram');

if (remoteDiagram) {
    fetch(remoteDiagram).then(
        r => {
            if (r.ok) {
                return r.text();
            }

            throw new Error(`Status ${r.status}`);
        }
    ).then(
        text => openDiagram(text)
    ).catch(
        err => {
            showMessage('error', `Failed to open remote diagram: ${err.message}`);

            openDiagram(initialDiagram);
        }
    );
} else {
    openDiagram(initialDiagram);
}

// really useful ???? as everything else doesn't seems to work correctly ?
/**
 * Quality of Service handler
 */
var qosCheck = document.getElementById('qos-check');

if (qosCheck && qosCheck.checked) {
    // --- QoS compatibility check --- //
    const modeling = modeler.get('modeling');
    const registry = modeler.get('elementRegistry');

    /* Importing Rule Engine instance */
    const R = qosCompatibilityCheck()

    modeler.on(['shape.removed'], (e) => {
        e.stopPropagation();
        e.preventDefault();
        const element = e.element;
    });


    /* Trigger QoS check every time a change occurs */
    modeler.on(['commandStack.element.updateProperties.postExecuted', 'commandStack.element.updateModdleProperties.postExecuted'], (e) => {
        e.stopPropagation();
        e.preventDefault();
        if (e.context.properties.di) return; // if background changes exit

        const element = e.context.element;
        if (!pubOrSub(element)) return; // if not signal node exit
        element.businessObject.incompatibilities = []; // reset incompatibilities
        if (!element.businessObject.eventDefinitions[0].signalRef) {
            modeling.setColor(registry.get(element.id), {
                fill: "#fff"
            }); // reset node color
            return;
        }
        const signalName = element.businessObject.eventDefinitions[0].signalRef.name; // get signal topic name

        var elements = registry.filter(function (element) {
            return (
                pubOrSub(element) &&
                element.businessObject.eventDefinitions[0].signalRef &&
                element.businessObject.eventDefinitions[0].signalRef.name == signalName
            );
        });

        for (const i in elements) {
            const source = elements[i];
            const type = pubOrSub(source);
            modeling.setColor(source, {
                fill: "#33bb77"
            }); // reset node color

            elements.forEach(target => {
                if (type == pubOrSub(target) || source.id == target.id) return;
                source.businessObject.incompatibilities = [];
                var communication = {
                    pub: type == "pub" ? source.businessObject : target.businessObject,
                    sub: type == "sub" ? source.businessObject : target.businessObject
                };
                R.execute(communication, (data) => {
                    if (data.result !== true) {
                        source.businessObject.incompatibilities.push({
                            "name": target.businessObject.name,
                            "reasons": data.reasons
                        });
                        modeling.setColor(source, {
                            fill: "#ff0000"
                        });
                        modeling.setColor(target, {
                            fill: "#ff0000"
                        });
                    }
                });
            });
        }
    });


    modeler.on(['element.click'], (e) => {
        let incompatibilities = e.element.businessObject.incompatibilities;
        if (!incompatibilities || incompatibilities.length == 0)
            document.getElementById("incompatibilities").innerHTML = null;
        else {
            let text = "";
            incompatibilities.forEach(e => {
                text += "<br>";
                text += (e.name + ": ");
                text += e.reasons;
            })
            document.getElementById("incompatibilities").innerHTML = "<div><b>INCOMPATIBILITIES!</b>" + text + "</div>";
        }
    });
}


/**
 * Call Activity handler
 */
if (document.getElementById('call-act-button')) {
    document.querySelector('#call-act-button').addEventListener('click', function (event) {
        storeCallAct(modeler)
    });
}
