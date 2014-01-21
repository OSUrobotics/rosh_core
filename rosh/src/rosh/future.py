#TODO: disabling until rospy can talk to multiple masters correctly
def __set_master(name='localhost'):
    """
    Switch the ROS master currently in use, e.g. master('prh'). If
    master() is called with no args, defaults to localhost.
    
    @param name: name of master (if running on default port), or full URI
    @type  name: str
    """
    
    # TODO: rospy needs to have built-in multimaster support for this
    # to actually work, or we need to get rid of the node singleton
    
    if name.startswith('http://'):
        ctx.master._reinit(name)
    else:
        # assume its a hostname
        ctx.master._reinit('http://%s:11311'%name)
        
    # update the system-wide environment 
    os.environ[roslib.rosenv.ROS_MASTER_URI] = ctx.master.master_uri
    return ctx.master.is_online()
    
class _Relay(object):

    def __init__(self, from_topic, to_topic):
        self.from_topic = from_topic
        self.to_topic   = to_topic

    def __call__(self, msg):
        # check for race condition
        to_topic = self.to_topic
        if to_topic is None:
            return

        # lazy-init new topic
        if to_topic._type_name is None:
            try:
                to_topic._init(msg._type)
            except:
                #TODO: log properly
                print >> sys.stderr, "FAILED to initialize, destroying relay [%s] | [%s]"%(self.from_topic, self.to_topic)
                self._unrelay()

        if msg._type == to_topic._type_name:
            to_topic(msg)
        else:
            #TODO: WARN user once
            pass

    def _unrelay(self):
        if self.from_topic is not None:
            self.from_topic._remove_subscriber_callback(self)
        self.from_topic = self.to_topic = None
        
    def __del__(self):
        self._unrelay()
        
class _FilteredRelay(_Relay):

    def __init__(self, from_topic, to_topic, filter_fn):
        super(_FilteredRelay, self).__init__(from_topic, to_topic)
        self.filter_fn = filter_fn

    def __call__(self, msg):
        msg = self.filter_fn(msg)
        super(_FilteredRelay, self).__call__(msg)

def relay(from_topic, to_topic, filter_fn=None):
    if filter_fn:
        relay_obj = _FilteredRelay(from_topic, to_topic, filter_fn)
    else:
        #TODO: replace with topic_tools launch
        relay_obj = _Relay(from_topic, to_topic)
    from_topic._add_subscriber_callback(relay_obj)
    return relay_obj

def unrelay(relay_obj):
    relay_obj._unrelay()
    
