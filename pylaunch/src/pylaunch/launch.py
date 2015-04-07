import roslib
import roslaunch.config as rc
import rosgraph.network
import roslaunch.xmlloader
import roslaunch.parent as roslaunch_parent
import roslaunch.rlutil as rlutil
import roslaunch.loader as rloader
import rosgraph.names as rn
import roslaunch.core as rr
import sys
import os.path as pt

class FileNotFoundException(Exception):
    pass

class PyRosLaunchItem(object):
    '''
        Abstract class for each item launched by pylaunch.
    '''

    def __init__(self):
        self.verbose = False

    def process(self, context, ros_launch_config):
        '''
            Modifies ROS launch innards to launch whichever item this
            is.

            Args:

            context (roslaunch.loader.LoaderContext)
            ros_launch_config (roslaunch.config.ROSLaunchConfig)
        '''
        raise RuntimeError('Unimplemented')

def py_types_to_string(v):
    '''
        Convert a Python value to its string representation for
        roslaunch.

        Args:

        v (int, bool, string, float)

        Returns:

        string representing this native python type
    '''
    if type(v) == bool:
        return str(v).lower()

    if type(v) == int or type(v) == float:
        return str(v)
    
    return v

class Include(PyRosLaunchItem):
    '''
        Represents an Include statement in roslaunch.

        Args:
            package_name (string)
            launch_file_name (string)
            params (dict): {'name': value (int, string, bool)}
                            (normally called 'args' in roslaunch for
                            an include directive)
    '''

    def __init__(self, package_name, launch_file_name, params=None):
        super(PyRosLaunchItem, self).__init__()
        l = roslib.packages.find_resource(package_name, launch_file_name)
        if len(l) == 0:
            raise FileNotFoundException("Include error. Package %s doesn't containt %s." % (package_name, launch_file_name))
        self.file_path = l[0]
        self.params = {} if params is None else params

    def process(self, context, ros_launch_config):
        context = rloader.LoaderContext(rn.get_ros_namespace(), self.file_path)
        child_ns = context.include_child(None, self.file_path)
        for k, v in self.params.iteritems():
            child_ns.add_arg(k, value=py_types_to_string(v))

        rloader.process_include_args(child_ns)

        parser = roslaunch.xmlloader.XmlLoader()
        launch = parser._parse_launch(self.file_path, verbose=self.verbose)
        ros_launch_config.add_roslaunch_file(self.file_path)
        parser._launch_tag(launch, ros_launch_config, filename=self.file_path)
        default_machine = parser._recurse_load(ros_launch_config, launch.childNodes, 
                                               child_ns,
                                               default_machine=None,
                                               is_core=False,
                                               verbose=self.verbose)

        rloader.post_process_include_args(child_ns)
        #print_context_vars(child_ns)


class Node(PyRosLaunchItem):
    '''
    Represents a ROS node to launch.

    Args:

        package_name (string)
        node_type (string): executable name in package.
        node_name (string): name for node after launching
        args (string): string to pass to executable
        params (dict): {'name': value (int, string, bool)}
        remaps (list of tuples): [('from_topic', 'to_topic')]
        namespace (string): namespace to stuff node into.
    '''

    def __init__(self, package_name, node_type, node_name, args=None, params=None, remaps=None, namespace='/'):
        super(PyRosLaunchItem, self).__init__()

        self.package_name = package_name
        self.node_type = node_type
        self.node_name = node_name

        self.params = params if params is not None else {}
        self.remaps = remaps if remaps is not None else []
        self.args = args
        self.namespace = namespace

    def process(self, context, ros_launch_config):
        #Add all our params to the ROSLaunchConfig
        param_ns = context.child(self.node_name)
        for name, value in self.params.iteritems():
            p = rr.Param(param_ns.ns + name, py_types_to_string(value))
            ros_launch_config.add_param(p, verbose=self.verbose)

        #Add to a LoaderContext verify that names are legal
        remap_ns = context.child('')
        for r in self.remaps:
            remap_ns.add_remap(r)

        #Create our node
        self.node = rr.Node(self.package_name, self.node_type,
                            self.node_name,
                            remap_args=remap_ns.remap_args(),
                            # Setting this pipes mutes the node's stdout.
                            #namespace=param_ns.ns, 
                            namespace=self.namespace, 
                            args=self.args) 
        ros_launch_config.add_node(self.node, self.verbose)


class PyRosLaunch(roslaunch_parent.ROSLaunchParent):
    """
        Used to write ROS launch files in Python.

        Example:

        configs = [Node("rospy_tutorials", "listener", "listener2",
                        params={'calibrate_time': False},
                        remaps=[('chatter', 'hello_topic')]),
                   Include('app_bringup', 'test3_hokuyo.launch', args={'hokuyo_num': '21'})]
        p = PyRosLaunch(config_list)
        p.start()
        raw_input("press enter to stop") # or p.spin(), p.spinOnce(), etc.
        p.shutdown()

        Args:

        config_list (list): list of PyRosLaunchItem objects.
        port (int): port number.
        verbose (bool): whether each node should be verbose.
    """

    def __init__(self, config_list, port=None, verbose=False):
        uuid = rlutil.get_or_generate_uuid(None, False)
        roslaunch_parent.ROSLaunchParent.__init__(self, uuid, None)

        ros_launch_config = rc.ROSLaunchConfig()

        if port:
            ros_launch_config.master.uri = rosgraph.network.create_local_xmlrpc_uri(port)

        xml_loader = roslaunch.xmlloader.XmlLoader()
        rc.load_roscore(xml_loader, ros_launch_config, verbose=verbose)
        xml_loader.root_context = rloader.LoaderContext(rn.get_ros_namespace(), 
                                    pt.splitext(pt.split(sys.argv[0])[-1])[0])

        for n in config_list:
            n.verbose = verbose
            n.process(xml_loader.root_context, ros_launch_config)

        ros_launch_config.assign_machines()
        self.config = ros_launch_config


class LaunchFileRunner(roslaunch_parent.ROSLaunchParent):
    """
        Runs a single ROS launch file.

        Example:

        p = LaunchFileRunner('my_package', 'my_launch_file.launch')
        p.start()
        raw_input("press enter to stop")
        p.shutdown()

        Args:

        package_name (string)
        launch_file_name (string)
    """

    def __init__(self, package_name, launch_file_name):
        uuid = rlutil.get_or_generate_uuid(None, False)
        file_path = roslib.packages.find_resource(package_name, launch_file_name)
        roslaunch_parent.ROSLaunchParent.__init__(self, uuid, file_path)


def launch(config_list):
    """
        Convenience func to run PyRosLaunch with a configuration list.
    """
    p = PyRosLaunch(config_list)
    p.start()
    p.spin()

def print_context_vars(context):
    print 'Variables'
    for n in context.arg_names:
        print n, ':', context.resolve_dict['arg'][n]

def run_include():
    config = rc.ROSLaunchConfig()
    inc = Include('app_bringup', 'test3_hokuyo.launch', {'hokuyo_num': '23'})
    inc.process(config)
    print 'node name', config.nodes[0].name

def run_py_ros_launch():
    p = PyRosLaunch(nodes)
    p.start()
    raw_input("press enter to stop")
    p.shutdown()
    



