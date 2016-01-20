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
import abc
import rospkg

def pkg_path(package_name):
    '''
        Locates a ROS package

        Args:
        package_name (string) name of package to locate.

        Returns:
        string path to ROS package

    '''
    ros_pack = rospkg.RosPack()
    return ros_pack.get_path(package_name)

def resource_path(package_name, filename):
    '''
        Locates a file known to ROS.
        
        Args:
        package_name (string) 
        filename (string) file in package to locate

        Returns:
        string path to file

    '''
    l = roslib.packages.find_resource(package_name, filename)
    if len(l) == 0:
        raise FileNotFoundException("Include error. Package %s doesn't containt %s." \
                                    % (package_name, filename))
    if len(l) > 1:
        raise RuntimeError("Multiple files named %s found in package %s" \
                            % (package_name, filename))
    return l[0]

class FileNotFoundException(Exception):
    pass

class PyRosLaunchItem(object):
    '''
        Abstract class for each item launched by pylaunch.
    '''
    __metaclass__ = abc.ABCMeta

    def __init__(self):
        self.verbose = False

    @abc.abstractmethod
    def process(self, loader, ros_launch_config):
        '''
            Modifies ROS launch innards to launch whichever item this
            is.

            Args:

            loader roslaunch.xmlloader.XmlLoader (context object is in
                    loader.root_context)
            ros_launch_config (roslaunch.config.ROSLaunchConfig)
        '''
        pass

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

    if type(v) in (int, float):
        return str(v)
    
    return v

class FInclude(PyRosLaunchItem):
    '''
        Represents an Include statement in roslaunch. Uses full paths.

        Args:
            file_path (string): path to file to include.

            remaps (list of tuples): [('from_topic', 'to_topic')]

            params (dict): {'name': value (int, string, bool)}
                            (normally called 'args' in roslaunch for
                            an include directive)

            rosparams (list): list of RosParam objects
    '''

    def __init__(self, file_path, remaps=None, params=None, rosparams=None):
        super(PyRosLaunchItem, self).__init__()
        if not pt.exists(file_path):
            raise RuntimeError("Included file %s does not exist." % file_path)

        self.file_path = file_path
        self.params = {} if params is None else params
        self.remaps = [] if remaps is None else remaps
        self.rosparams = rosparams if rosparams is not None else []
        

    def process(self, loader, ros_launch_config):
        context = rloader.LoaderContext(rn.get_ros_namespace(), self.file_path)
        child_ns = context.include_child(None, self.file_path)
        for r in self.remaps:
            context.add_remap(r)

        for k, v in self.params.iteritems():
            child_ns.add_arg(k, value=py_types_to_string(v))

        for rp in self.rosparams:
            rp.set_namespace(rn.ns_join(param_ns, rp.get_namespace()))
            rp.process(ros_launch_config)

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

class Include(FInclude):
    '''
        Represents an Include statement in roslaunch. Needs only package name and filename.

        Args:
            package_name (string)

            launch_file_name (string)

            remaps (list of tuples): [('from_topic', 'to_topic')]

            params (dict): {'name': value (int, string, bool)}
                            (normally called 'args' in roslaunch for
                            an include directive)

            rosparams (list): list of RosParam objects
    '''
    def __init__(self, package_name, launch_file_name, remaps=None, 
                 params=None, rosparams=None):
        filename = resource_path(package_name, launch_file_name)
        super(Include, self).__init__(filename, remaps, params, rosparams)


class RosParam(PyRosLaunchItem):
    '''
        Represents a rosparam statement in roslaunch.

        Args:

            param_file (string) yaml file to load

            command (string) one of of 'load', 'dump', or 'delete'

            namespace (string) scope the params to a namespace
    '''
    def __init__(self, param_file, command='load', namespace='/'):
        super(PyRosLaunchItem, self).__init__()
        self.command = command
        self.param_file = param_file
        self.namespace = namespace

    def get_namespace(self):
        return self.namespace

    def set_namespace(self, ns):
        self.namespace = ns

    def process(self, loader, ros_launch_config):
        param = rn.ns_join('', self.namespace)
        loader.load_rosparam(loader.root_context, ros_launch_config, 
                self.command, param, self.param_file, '')

class Param(PyRosLaunchItem):
    '''
        Ros param command.
    '''
    def __init__(self, name, value=None, ptype='auto', command=None):
        if value is None and (ptype is None or command is None):
            raise RuntimeError('Either provide value or ptype and command')

        self.name = name
        self.value = value
        self.ptype = ptype
        self.command = command

    def process(self, loader, ros_launch_config):
        if self.value is not None:
            value = loader.param_value(self.verbose, self.name, self.ptype, 
                                        py_types_to_string(self.value), 
                                        None, None, None)
        else:
            value = loader.param_value(self.verbose, self.name, self.ptype, 
                                       value=None, textfile=None, binfile=None, 
                                       command=self.command)

        ros_launch_config.add_param(rr.Param(self.name, value), verbose=self.verbose)

class Node(PyRosLaunchItem):
    '''
    Represents a ROS node to launch.

    Args:

        package_name (string)

        node_type (string): executable name in package.

        node_name (string): name for node after launching

        args (string): string to pass to executable

        params (dict): {'name': value (int, string, bool)}

        rosparams (list): list of RosParam objects
        
        remaps (list of tuples): [('from_topic', 'to_topic')]

        namespace (string): namespace to stuff node into.

        respawn (bool)

        output (string) either 'screen' or 'log'

        launch_prefix (string) for things like gdb, valgrind, sudo, etc.
    '''

    def __init__(self, package_name, node_type, node_name, 
                 args=None, params=None, rosparams=None, remaps=None, 
                 namespace='/', respawn=False, output=None, launch_prefix=None):
        super(PyRosLaunchItem, self).__init__()

        self.package_name = package_name
        self.node_type = node_type
        self.node_name = node_name

        self.rosparams = rosparams if rosparams is not None else []
        self.params = params if params is not None else {}
        self.remaps = remaps if remaps is not None else []
        self.args = args
        self.namespace = namespace
        self.respawn = respawn
        self.output = output
        self.launch_prefix = launch_prefix

    def process(self, loader, ros_launch_config):
        context = loader.root_context
        #Add all our params to the ROSLaunchConfig
        param_ns = context.child(self.node_name)
        for name, value in self.params.iteritems():
            loader_value = loader.param_value(self.verbose, name, 'auto', 
                                        py_types_to_string(value), 
                                        None, None, None)
            p = rr.Param(param_ns.ns + name, loader_value)
            ros_launch_config.add_param(p, verbose=self.verbose)

        for rp in self.rosparams:
            rp.set_namespace(rn.ns_join(param_ns.ns, rp.get_namespace()))
            rp.process(loader, ros_launch_config)

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
                            args=self.args, 
                            respawn=self.respawn,
                            output=self.output, 
                            launch_prefix=self.launch_prefix)
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
            n.process(xml_loader, ros_launch_config)

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
    



