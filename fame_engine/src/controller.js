'use strict';

const { Engine } = require('bpmn-engine');
const bent = require('bent');
const rclnodejs = require('rclnodejs');
const { Parameter, ParameterDescriptor, ParameterType } = rclnodejs;
const { EventEmitter } = require('events');
const listener = new EventEmitter();
const fs = require('fs');
var convert = require('xml-js');
const path = require('path');

var topic_dict = {};

var source = '';
var engine_env = {};
var process_path = 'process/';
var process_dict = {};

//console.log(__dirname.split('/install')[0] + '/process')

function stripNS(obj) {
    if (Array.isArray(obj)) {
        return obj.map(stripNS);
    } else if (typeof obj === 'object' && obj !== null) {
        const newObj = {};
        for (const [key, value] of Object.entries(obj)) {
            const cleanKey = key.includes(':') ? key.split(':')[1] : key;
            newObj[cleanKey] = stripNS(value);
            //console.log("striping " + cleanKey);
        }
        return newObj;
    }
    return obj;
}

// --- BUNDLING HELPERS ---
const os = require('os');

function resolveBaseDir(pth) {
  // 1) absolu ou ~
  const maybeAbs = expandUser(pth);
  if (path.isAbsolute(maybeAbs)) return maybeAbs;

  // 2) candidats quand c'est relatif
  const candidates = [
    path.join(process.cwd(), pth),           // CWD (lancement)
    path.join(__dirname, pth),               // dossier du script (dist/)
    path.join(__dirname, '..', pth),         // parent du script (share/fame_engine/)
  ];

  for (const c of candidates) {
    try {
      if (fs.existsSync(c) && fs.statSync(c).isDirectory()) return c;
    } catch (_) {}
  }

  // fallback: retourne tel quel (laissera l'erreur explicite en aval)
  return pth;
}


function expandUser(p) {
  return (typeof p === 'string' && p.startsWith('~'))
    ? path.join(os.homedir(), p.slice(1))
    : p;
}

function escapeRe(s) {
  return s.replace(/[.*+?^${}()|[\]\\]/g, '\\$&');
}

function safeListBpmnFiles(dir) {
  const out = [];
  try {
    const items = fs.readdirSync(dir, { withFileTypes: true });
    for (const it of items) {
      const full = path.join(dir, it.name);
      if (it.isFile() && it.name.endsWith('.bpmn')) out.push(full);
      else if (it.isDirectory()) {
        try {
          const sub = fs.readdirSync(full, { withFileTypes: true });
          for (const s of sub) {
            const f = path.join(full, s.name);
            if (s.isFile() && s.name.endsWith('.bpmn')) out.push(f);
          }
        } catch (_) {}
      }
    }
  } catch (_) {}
  return out;
}

function extractInnerDefinitions(xml, fileLabel = '') {
  const m = xml.match(/<bpmn:definitions[\s\S]*?>([\s\S]*?)<\/bpmn:definitions>/);
  if (!m) throw new Error(`Pas de <bpmn:definitions> dans: ${fileLabel || 'source in-memory'}`);
  return m[1];
}

function usesNamespace(xml, nsPrefixWithColon) {
  const re = new RegExp(`\\b${nsPrefixWithColon.replace(':', '\\:')}`);
  return re.test(xml);
}

function makeBundle(xmlStrings) {
  // Ajoute automatiquement camunda si au moins un document l’utilise.
  const needsCamunda = xmlStrings.some(x => usesNamespace(x, 'camunda:'));

  const header = [
    `<?xml version="1.0" encoding="UTF-8"?>`,
    `<bpmn:definitions xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance"`,
    `  xmlns:bpmn="http://www.omg.org/spec/BPMN/20100524/MODEL"`,
    `  xmlns:bpmndi="http://www.omg.org/spec/BPMN/20100524/DI"`,
    `  xmlns:dc="http://www.omg.org/spec/DD/20100524/DC"`,
    `  xmlns:di="http://www.omg.org/spec/DD/20100524/DI"${needsCamunda ? `\n  xmlns:camunda="http://camunda.org/schema/1.0/bpmn"` : ''}`,
    `  targetNamespace="http://bpmn.io/schema/bpmn">`
  ].join('\n');

  const body = xmlStrings.map((xml, i) => extractInnerDefinitions(xml, `bundle#${i}`)).join('\n');
  return `${header}\n${body}\n</bpmn:definitions>`;
}

// ---  ---
// --- CALL ACTIVITY RESOLUTION ---

function resolveCalledFile(blockName, node) {
  if (!blockName) throw new Error('[resolveCalledFile] blockName vide');
  const baseDir = resolveBaseDir(process_path);       // ex: .../share/fame_engine/process/
  const ns = (node && typeof node.namespace === 'function')
    ? node.namespace().replace(/\//g, '')
    : '';

  // Candidats EXACTEMENT selon ta règle
  const candA = path.join(baseDir, `${blockName}.bpmn`);
  const candB = ns ? path.join(baseDir, blockName, `${ns}.bpmn`) : null;

  // (optionnel mais pratique) variante tolérante: process/<bloc>/<bloc>.bpmn
  const candC = path.join(baseDir, blockName, `${blockName}.bpmn`);

  // Debug utile
  // console.log('[resolveCalledFile]', { baseDir, candA, candB, candC });

  if (fs.existsSync(candA)) return candA;
  if (candB && fs.existsSync(candB)) return candB;
  if (fs.existsSync(candC)) return candC;

  // Trace claire si rien trouvé
  console.error('[resolveCalledFile] NOT FOUND for block:', blockName);
  console.error('  tried:', candA, candB || '(no ns)', candC);
  throw new Error(`[resolveCalledFile] Introuvable: ${blockName} (baseDir: ${baseDir})`);
}


// Utilitaires

// function safeListBpmnFiles(dir) {
//   // récursif light: parcours dir et sous-dossiers 1 niveau (suffit souvent)
//   const acc = [];
//   try {
//     const items = fs.readdirSync(dir, { withFileTypes: true });
//     for (const it of items) {
//       const full = path.join(dir, it.name);
//       if (it.isFile() && it.name.endsWith('.bpmn')) acc.push(full);
//       else if (it.isDirectory()) {
//         // un niveau de profondeur
//         try {
//           const sub = fs.readdirSync(full, { withFileTypes: true });
//           for (const s of sub) {
//             const f = path.join(full, s.name);
//             if (s.isFile() && s.name.endsWith('.bpmn')) acc.push(f);
//           }
//         } catch (_) {}
//       }
//     }
//   } catch (_) {}
//   return acc;
// }

// function escapeRe(s) {
//   return s.replace(/[.*+?^${}()|[\]\\]/g, '\\$&');
// }

// Parcours le XML courant, trouve les Call Activities, lit leurs fichiers, enregistre les sources
function writeProcess(source_process, node) {
    const conversion = convert.xml2json(source_process, { compact: true, spaces: 2 }).replace(/'/g, '"');
    const conversion_obj = stripNS(JSON.parse(conversion));

    const defs = conversion_obj?.definitions;
    if (!defs) return;

    const processObj = defs.process; // peut être array ou objet
    if (!processObj) return;

    const processes = Array.isArray(processObj) ? processObj : [processObj];

    for (const p of processes) {
        const caObjs = p.callActivity;
        if (!caObjs) continue;

        const calls = Array.isArray(caObjs) ? caObjs : [caObjs];
        for (const ca of calls) {
            const attrs = ca?._attributes || {};
            // PRIORITÉ: calledElement (ID BPMN), FALLBACK: name (convention fichier)
            // const calledElementId = attrs.calledElement && String(attrs.calledElement).trim();
            // const calledByName     = attrs.name && String(attrs.name).trim();
            // const calledElementId = attrs.calledElement && String(attrs.calledElement).trim();
            // const calledByName     = attrs.name && String(attrs.name).trim();
            // const calledKey = (calledElementId || calledByName);


            // // const calledKey = (calledElementId || calledByName);
            // if (!calledKey) continue;

            // const ca_file = resolveCalledFile(calledKey, node);
            // const ca_source = fs.readFileSync(ca_file, 'utf8');

            // if (!process_dict[calledKey]) {
            //     process_dict[calledKey] = [ca_source, false];
            // }
            // const attrs = ca?._attributes || {};
            const blockName = attrs.name && String(attrs.name).trim();
            if (!blockName) continue;

            const ca_file = resolveCalledFile(blockName, node);
            const ca_source = fs.readFileSync(ca_file, 'utf8');

            if (!process_dict[blockName]) {
                process_dict[blockName] = [ca_source, false];
            }
        }
    }

    // Récursif : aller chercher les Call Activities des sous-process lus
    Object.keys(process_dict).forEach(key => {
        if (!process_dict[key][1]) {
        process_dict[key][1] = true;
        writeProcess(process_dict[key][0], node);
        }
    });
}
function merge_callActivity(rootXml, node) {
  // 1) Indexer tous les sous-process via writeProcess (remplit process_dict)
  process_dict = {}; // reset pour éviter des reliquats
  writeProcess(rootXml, node);

  // 2) Construire la liste des sources à fusionner : racine + tous les sous-process
  const allSources = [rootXml, ...Object.values(process_dict).map(([xml]) => xml)];

  // 3) Bundler dans une seule <bpmn:definitions> (option B)
  // ⚠️ On NE repasse PAS par xml-js ici (sinon on perd les prefixes).
  const bundled = makeBundle(allSources);

  return bundled;
}

// main execution
rclnodejs.init().then(() => {
    //start ROS node
    const node = rclnodejs.createNode('engine_node');
    const geometry_msgs = rclnodejs.require('geometry_msgs');
    const tello_msgs = rclnodejs.require('tello_msgs');

    try {
        rclnodejs.createMessageObject('geometry_msgs/msg/Twist');
        rclnodejs.createMessageObject('tello_msgs/srv/TelloAction_Request');
        console.log('[controller] ROS types loaded');
    } catch (e) {
        console.error('[controller] Type loading failed:', e);
    }

    // Expose dans services pour tes scripts BPMN
    const Types = {
        Twist: geometry_msgs.msg.Twist,
        TelloAction: tello_msgs.srv.TelloAction,
    };

    // # Options for cmd 
    function getArg(flag, def = undefined) {
        const i = process.argv.indexOf(flag);
        return i >= 0 && i + 1 < process.argv.length ? process.argv[i + 1] : def;
    }
    const bpmn_name = getArg('--bpmn', node.namespace().replace('/', ''));
    // const userName = getArg('--name', 'world');
    // const mode     = getArg('--mode', 'normal');

    // // Déclarations + valeurs par défaut
    // node.declareParameter('bpmn_name', node.namespace().replace('/', '_'));
    // // node.declareParameter('frame_id', 'map');
    // // node.declareParameter('enabled', true);

    // // Lecture
    // const bpmn_name = node.getParameter('bpmn_name').value;
    // // const frameId = node.getParameter('frame_id').value;
    // // const enabled = node.getParameter('enabled').value;

    // node.declareParameters([
    //         new Parameter('bpmn_name', ParameterType.PARAMETER_STRING, node.namespace().replace('/', '')),
    //         // new Parameter('rate', ParameterType.PARAMETER_INTEGER, 10),
    //         // new Parameter('frame_id', ParameterType.PARAMETER_STRING, 'map'),
    //     ], [
    //         new ParameterDescriptor('bpmn_name', ParameterType.PARAMETER_STRING, ''),
    //         // new ParameterDescriptor('rate', ParameterType.PARAMETER_INTEGER, 'Hz'),
    //         // new ParameterDescriptor('frame_id', ParameterType.PARAMETER_STRING, 'TF frame'),
    // ]);
    // const bpmn_name = node.getParameter('bpmn_name').value;

    // var process_name = node.namespace().replace('/', '');
    // source = (fs.readFileSync(process_path + process_name + '.bpmn', 'utf8'));
    // source = fs.readFileSync(path.resolve(process.cwd(), process_path.replace(/^~\//, ''), `${bpmn_name}.bpmn`), 'utf8');
    // source = merge_callActivity(source, node);
    const baseDir = resolveBaseDir(process_path);
    source = fs.readFileSync(path.join(baseDir, `${bpmn_name}.bpmn`), 'utf8');
    source = merge_callActivity(source, node);

// (debug utile)
// console.log('[BPMN baseDir]', baseDir);
// fs.writeFileSync(path.join(baseDir, '__bundle_debug__.bpmn'), source);


    // (optionnel) pour debug
    // fs.writeFileSync(path.resolve(process.cwd(), 'bundle.bpmn'), source, 'utf8');

    //fs.writeFileSync(process_path+'res.bpmn', source);

    var tstart = 0;
    var tfinish = 0;

    // initialization of the engine
    const engine = Engine({
        name: 'fame',
        source
    });
    /*
        listener.on('flow.take', (flow) => {
            console.log(`flow.take <${flow.id}> was taken`);
        });
     */

    // at every block start
    listener.on('activity.start', (activity) => {
        if (tstart == 0) tstart = activity.messageProperties.timestamp;
        handleDataObj(activity);
        // console.log(`activity.start <${activity.id}> was taken`);
    });

    // at every block end
    listener.on('activity.end', (activity) => {
        tfinish = activity.messageProperties.timestamp;
        // add activity variables to global ones
        addVars(activity.environment.variables);
        engine_env = engine.environment;
        //console.log('HERE:', engine_env.variables);
        // console.log(`activity.end <${activity.id}> was released`);
    });

    /* 
    listener.on('activity.wait', (wait) => {
        console.log(`wait <${wait.id}> was taken`);
    });

    listener.on('activity.throw', (throwev) => {
        console.log(`throw <${throwev.id}> was taken`);
    });

    listener.on('activity.error', (errorev) => {
        console.log(`error <${errorev.id}> was taken`);
    });
    */

    function addVars(var_activity) {
        Object.keys(var_activity).forEach(element => {
            if (element != 'ros_node' && element != 'fields' && element != 'content' && element != 'properties') {
                if (!(element in Object.keys(engine.environment.variables))) {
                    var vs = new Object();
                    vs[element] = var_activity[element];
                    engine.environment.assignVariables(vs)
                }
            }
        });
    }

    /**
     * Management of signal throwing // CATS: sus, seems to not work, to be checked // redo comments seems 
     */
    engine.broker.subscribeTmp('event', 'activity.signal', (routingKey, msg) => { // routingKey = activity.signal
        let topic_name = msg.content.name;
        console.log("signal activity for:", topic_name);

        let message_type;
        let message_payload;
        let check = false;
        //console.log(msg);
        const regexpr = /\${(.*?)\}/g; // all variables are identified through ${...}
        // console.log(topic_dict);
        // for every ros topic FaMe uses
        for (let key in topic_dict) { // what is topic_dict ? => the dictionary with every ros topic that FaMe uses
            // if the name of the topic correspond to the one from the signal go on 
            if (key === topic_name) { // mhm not really correct but ok
                // get message type from topi_dict
                message_type = topic_dict[key][0];
                // and its payload (the data sent by the signal)
                message_payload = topic_dict[key][1];
                // get the vars from the payload
                var tempvar = message_payload.match(regexpr);
                // check if there are variables that needs a value assignment
                if (tempvar) {
                    // for every varables that needs to be assigned
                    for (let i = 0; i < tempvar.length; i++) {
                        var val = tempvar[i];
                        if (val.startsWith('$')) {
                            var variable = val.substring(2, val.length - 1); // removes ${}
                            //console.log(engine_env.variables);
                            // set value to the value stored inside the engine's dictionnary
                            var value = engine_env.variables[variable];
                            //console.log(value);
                            // change `${var_name}` into `var_value`
                            message_payload = message_payload.replace(val, value); // replace variable with value
                            topic_dict[key][1] = message_payload; // update topic dictionary
                        }
                    }
                }
                check = true;
                break;
            }
        }
        // Publish ros topic
        // console.log(check);
        if (check) {
            engine.execution.signal(msg.content.message, { ignoreSameDefinition: true });
            console.log(`Publishing message on ${topic_name}: ` + message_payload);
            const publisher = node.createPublisher(message_type, '/' + topic_name);
            var message_obj = JSON.parse(message_payload);  // conversion from string to obj
            publisher.publish(message_obj);
        }
    }, { noAck: true });


    const clients = new Map(); // (voir étape 2 pour remplir)
    engine.execute({
        listener,
        variables: {
            ros_node: node
        },
        services: {
            get: bent('json'),
            set,

            // to be able to print in the console
            console,

            // expose rclnodejs and the node to the sandbox
            rclnodejs,
            ros_node: node,
            clients, // optionnel mais recommandé (voir ci-dessous)
            Types,

            // timers Node déjà exposés si besoin
            setTimeout,
            clearTimeout,
            setInterval,
            clearInterval,

        },
        moddleOptions: {
            camunda: require('camunda-bpmn-moddle/resources/camunda'),
        },
        extensions: {
            camunda: camundaExtProperties,
        }
    });

    engine.on('end', (execution) => {
        // console.log('Ended:', process_name);
        console.log('Ended:', bpmn_name);
        // you might want to change this, but currently it is more annoying then anything else for me
        console.log("killing the process");
        process.kill(process.pid);
    });

    // example of a simple implementation of a function inside the engine
    function set(activity, name, value) {
        activity.logger.debug('set', name, 'to', value);
    }


    //function used during signal interpretation to extract camunda properties // doesn't seems to work as intended
    /**
     * Manages camunda external properties // stfu it is not that
     * @param {*} activity 
     */
    function camundaExtProperties(activity) {
        // if there is no activity.behaviour.extensionElements: pass (why .behaviour ???)
        if (!activity.behaviour.extensionElements) return;
        let msg_type; // message type
        let ref_topic; // topic name
        let msg_payload; // massage payload

        // console.log(activity)
        // console.log(activity.activityDefinition)
        // console.log(activity.behaviour)
        // console.log(activity.behaviour['$type'])
        // console.log(activity.behaviour.name)
        // console.log(activity.behaviour.extensionElements)
        // console.log(activity.behaviour.extensionElements.$type)
        if (!activity.behaviour.extensionElements.values) return;
        for (const extn of activity.behaviour.extensionElements.values) {
            // only do the following operation on the 'properties' block
            if (extn.$type === 'properties') {
                ref_topic = activity.name;
                let prop = extn.$children; // properties data
                msg_type = prop[0].value; // TO FIX -> non ha controlli // CATS: ThFu is that ???? that's why it doesn't work
                // if it is a throw signal
                if (prop.length > 1) {
                    msg_payload = prop[1].value;
                    topic_dict[ref_topic] = [msg_type, msg_payload]                 // save properties parameters in the topic dictionary
                } else { // it is a catch
                    console.log('Subscribed to: ', ref_topic);
                    // added '/' to avoid remap of topics
                    node.createSubscription(msg_type, '/' + ref_topic, (msg) => { //create ROS subscription
                        console.log(`Received message: `, msg);
                        //activity.environment.assignVariables();
                        Object.keys(msg).forEach(element => {
                            // assign to global variables the payload of the signal
                            const find = Object.keys(activity.environment.variables).find(v => v.startsWith(element));
                            if (find) { // check if there is a matching variable
                                var value = msg[element];
                                activity.environment.variables[find] = value;
                            }
                        });
                        activity.getApi().sendApiMessage('signal'); // forces signal catching
                    });
                }
            }
        }
    }

    /**
     * Assigns data object values to global variables
     * @param {*} activity 
     */
    function handleDataObj(activity) {
        if (!activity.owner.behaviour.dataInputAssociations) {
            if (!activity.owner.behaviour.dataOutputAssociations) return; // check if there are associated data objects
        }
        // conversion from xml to object
        var xml = source;
        var conversion = convert.xml2json(xml, { compact: true, spaces: 4 });
        conversion = conversion.replace(/'/g, '"');
        var conversion_obj_raw = JSON.parse(conversion);
        var conversion_obj = stripNS(conversion_obj_raw);
        // data objects extraction
        var processObjs = conversion_obj['definitions']['process'];
        var dataObjs = [];
        if (processObjs.length) {
            for (let i in processObjs) {
                var dObj = processObjs[i]['dataObjectReference'];
                if (dObj) {
                    if (dObj.length) {
                        for (let j in dObj) {
                            dataObjs.push(dObj[j]);
                        }
                    } else {
                        dataObjs.push(dObj);
                    }
                }
            }
        }

        // activities data objects extraction
        if (!activity.owner.behaviour.dataInputAssociations) {
            var act_obj = activity.owner.behaviour.dataOutputAssociations[0].behaviour.targetRef.id;
        } else
            var act_obj = activity.owner.behaviour.dataInputAssociations[0].behaviour.sourceRef.id;
        for (let da in dataObjs) {
            var obj = dataObjs[da]
            var obj_id = obj._attributes.id;
            if (act_obj === obj_id) {
                var variable = new Object();
                // if there are values assigned to the variable
                if (obj['extensionElements']) {
                    var properties = obj['extensionElements']['properties']['property'];
                    //console.log(properties);
                    if (properties.length > 1) {
                        for (let p_index in properties) {
                            var p = properties[p_index]._attributes;
                            // if (!p.value.startsWith('$'))
                            variable[p.name] = p.value;
                        }
                    } else {
                        var p = properties._attributes;
                        //if (!p.value.startsWith('$'))
                        variable[p.name] = p.value;
                    }

                    // add data object variables to the global environment
                    activity.environment.assignVariables(variable);

                    Object.keys(engine.environment.variables).forEach(element => {
                        if (!(element in Object.keys(activity.environment.variables))) {
                            var v = new Object();
                            v[element] = (engine.environment.variables[element]);
                            // console.log(v);
                            activity.environment.assignVariables(v);
                        }
                    });
                    engine.environment.assignVariables(variable);
                }
            }
        }
    }

    rclnodejs.spin(node);
});

