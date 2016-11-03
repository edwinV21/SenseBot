var Intruder     = require('../models/intruder');
var Gas     = require('../models/gas');
var Temperature     = require('../models/temperature');
var http = require('http');
var dirA,dirB,dirI,dirD;

var options = {
  host: '192.168.1.67',
  path: '/?flagSens=0&dirA=1&dirB=0&dirI=0&dirD=0'
};

callback = function(response) {
  var str = '';

  //another chunk of data has been recieved, so append it to `str`
  response.on('data', function (chunk) {
    str += chunk;
  });

  //the whole response has been recieved, so we just print it out here
  response.on('end', function () {
    console.log(str);
  });
}

module.exports = function(app, express) {
	var router = express.Router();

	// test route to make sure everything is working (accessed at GET http://localhost:8080/api)
	router.get('/', function(req, res) {
	    res.json({ message: 'hooray! welcome to our api!' });   
	});

	// Gas Endpoint (accessed at http://localhost:8080/api/gas)
	router.route('/gas')
    // POST gas
    .post(function(req, res) {
        var gas = new Gas();
        gas.gasLevel = req.body.gasLevel;
        gas.danger = req.body.danger;
        // save it
        gas.save(function(err) {
            if (err)
                res.send(err);
            console.log('post gas');
            console.log(req.body);
            res.json({ message: 'Gas created!' });
        })
    })
    // GET gas
    .get(function(req, res) {
        Gas.
        find({}).
        limit(5).
        sort({ time: -1 }).
        then(function(err,docs) {
            if (err)
                res.send(err);

            res.json(docs);
        });
   });

    // Temperature Endpoint (accessed at http://localhost:8080/api/temp)
	router.route('/temp')
    // POST temp
    .post(function(req, res) {
        var temp = new Temperature();
        temp.temperature = req.body.temperature;
        temp.danger = req.body.danger;
        // save it
        temp.save(function(err) {
            if (err)
                res.send(err);
            console.log('post temp');
            console.log(req.body);
            res.json({ message: 'Temp created!' });
        })
    })
    // GET temp
    .get(function(req, res) {
        Temperature.
        find({}).
        limit(5).
        sort({ time: -1 }).
        then(function(err,docs) {
            if (err)
                res.send(err);

            res.json(docs);
        });
    });

    // Intruder Endpoint (accessed at http://localhost:8080/api/temp)
	router.route('/intruder')
    // POST intr
    .post(function(req, res) {
        var intruder = new Intruder();
        // save it
        intruder.save(function(err) {
            if (err)
                res.send(err);
            console.log('post intruder');
            console.log(req.body);
            res.json({ message: 'Intruder created!' });
        })
    })
    // GET intr
    .get(function(req, res) {
        Intruder.
        find({}).
        limit(5).
        sort({ time: -1 }).
        then(function(err,docs) {
            if (err)
                res.send(err);

            res.json(docs);
        });
    });

    //Moving car Endpoint
    router.route('/move/:dir')
    .post(function (req,res) {
        var dirP = req.params.dir;
        dirA=0;dirB=0;dirI=0;dirD=0;
        if(dirP==='up')dirA=1;
        else if(dirP==='down')dirB=1;
        else if(dirP==='left')dirI=1;
        else if(dirP==='down')dirD=1;
        options.path = "/?flagSens=0&dirA="+dirA+"&dirB="+dirB+"&dirI="+dirI+"&dirD="+dirD;
        http.request(options, callback).end();
        res.send('ok');
        // var uri =  "192.168.1.67:80/?dirA="+dirA+"&dirB="+dirB+"&dirI="+dirI+"&dirD="+dirD;
    });

    router.route('/sense/:flag')
    .post(function (req,res) {
        var flagSens = req.params.flag;
        if(flagSens) options.path = "/?flagSens=1&dirA=0&dirB=0&dirI=0&dirD=0";
        else options.path = "/?flagSens=1&dirA=0&dirB=0&dirI=0&dirD=0";
        http.request(options, callback).end();
        res.send('ok');
        // var uri =  "192.168.1.67:80/?dirA="+dirA+"&dirB="+dirB+"&dirI="+dirI+"&dirD="+dirD;
    });
	return router;
};