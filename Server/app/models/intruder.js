var mongoose     = require('mongoose');
var Schema       = mongoose.Schema;

var IntruderSchema   = new Schema({
	danger: Boolean,
    time: { type: Date, default: Date.now }
});

module.exports = mongoose.model('Intruder', IntruderSchema);