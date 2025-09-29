var mongoose = require('mongoose');
var Schema = mongoose.Schema;


var rosMsgSchema = new Schema({
    type: { type: String, required: true, unique: true },
    class: { type: String, required: true },
    payload: { type: mongoose.SchemaTypes.Mixed, required: true },
},
    {
        versionKey: false
    });


var RosMsg = mongoose.model('RosMsg', rosMsgSchema);
module.exports = RosMsg;