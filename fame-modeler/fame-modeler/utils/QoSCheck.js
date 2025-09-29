import { isSignalSupported } from "./EventDefinitionUtil";
import qosModdleDescriptor from '../descriptors/qos';

/* Creating Rule Engine instance */
export function qosCompatibilityCheck() {
    const R = new NodeRules();

    /* Add QoS compatibility rule */
    const rule = {
        condition: (R, qos) => {
            R.when(
                qos.pub.reliability < qos.sub.reliability ||
                qos.pub.durability > qos.sub.durability ||
                qos.pub.deadline > qos.sub.deadline ||
                qos.pub.liveliness < qos.sub.liveliness ||
                qos.pub.leaseDuration > qos.sub.leaseDuration
            );
        },
        consequence: (R, qos) => {
            qos.result = false;
            qos.reasons = [];
            if (qos.pub.reliability < qos.sub.reliability) qos.reasons.push("Reliability");
            if (qos.pub.durability > qos.sub.durability) qos.reasons.push("Durability");
            if (qos.pub.deadline > qos.sub.deadline) qos.reasons.push("Deadline");
            if (qos.pub.liveliness < qos.sub.liveliness) qos.reasons.push("Liveliness");
            if (qos.pub.leaseDuration > qos.sub.leaseDuration) qos.reasons.push("Lease duration");
            R.stop();
        },
    };

    /* Register QoS rule */
    R.register(rule);

    return R;
}

/* Evaluate if signal is publisher or subscriber */
export function pubOrSub(el) {
    if (!isSignalSupported(el)) return null;
    else if (el.type == "bpmn:StartEvent" || el.type == "bpmn:IntermediateCatchEvent") return "sub";
    else if (el.type == "bpmn:EndEvent" || el.type == "bpmn:IntermediateThrowEvent") return "pub";
    return null;
  }