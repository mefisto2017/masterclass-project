let vueApp = new Vue({
    el: "#vueApp",
    computed: {
        ws_address: function() {
            return `${this.rosbridge_address}`
        },
    },
    data: {
        // ros connection
        connected: false,
        ros: null,
        logs: [],
        loading: false,
        rosbridge_address: 'wss://i-00cbdc40fcccd3514.robotigniteacademy.com/7e4d6577-22bd-40b2-b93e-1dab1f84d000/rosbridge/', 
        port: '9090',
        // page content
        menu_title: 'Connection',
        // dragging data
        dragging: false,
        x: 'no',
        y: 'no',
        dragCircleStyle: {
            margin: '0px',
            top: '0px',
            left: '0px',
            display: 'none',
            width: '20px',
            height: '20px',
        },
        // joystick valules
        joystick: {
            vertical: 0,
            horizontal: 0,
        },
        // publisher
        pubInterval: null,
        // map
        mapViewer: null,
        mapGridClient: null,
        interval: null,
        // 3D stuff
        viewer: null,
        tfClient: null,
        urdfClient: null,
        // Action
        goal: null,
        action: {
            goal: { position: {x: 0, y: 0, z: 0} },
            feedback: { position: 0, state: 'idle' },
            result: { success: false },
            status: { status: 0, text: '' },
        }
    },
    methods: {
        connect: function() {
            // define ROSBridge connection object
            this.loading = true
            this.ros = new ROSLIB.Ros({
                url: this.rosbridge_address
            })
            // define callbacks
            this.ros.on('connection', () => {
                this.logs.unshift((new Date()).toTimeString() + ' - Connected!')
                this.connected = true
                this.loading = false
                console.log('Connection to ROSBridge established!')
                // Joystick pub
                this.pubInterval = setInterval(this.publish, 100)
                // Camera
                this.setCamera()
                // Map
                this.mapViewer = new ROS2D.Viewer({
                    divID: 'map',
                    width: 400,
                    height: 400
                })
                // Setup the map client.
                this.mapGridClient = new ROS2D.OccupancyGridClient({
                    ros: this.ros,
                    rootObject: this.mapViewer.scene,
                    continuous: true,
                })
                // Scale the canvas to fit to the map
                this.mapGridClient.on('change', () => {
                    scale = 0.2
                    this.mapViewer.scaleToDimensions(this.mapGridClient.currentGrid.width*scale, this.mapGridClient.currentGrid.height*scale);
                    this.mapViewer.shift(this.mapGridClient.currentGrid.pose.position.x*scale, this.mapGridClient.currentGrid.pose.position.y*scale)
                })
                // 3D stuff
                this.setup3DViewer()
            })
            this.ros.on('error', (error) => {
                this.logs.unshift((new Date()).toTimeString() + ` - Error: ${error}`)
            })
            this.ros.on('close', () => {
                this.logs.unshift((new Date()).toTimeString() + ' - Disconnected!')
                this.connected = false
                this.loading = false
                console.log('Connection to ROSBridge was closed!')
                clearInterval(this.pubInterval)
                document.getElementById('robotCamera').innerHTML = ''
                document.getElementById('map').innerHTML = ''
                this.unset3DViewer()
            })
        },
        publish: function() {
            let topic = new ROSLIB.Topic({
                ros: this.ros,
                name: '/cmd_vel',
                messageType: 'geometry_msgs/Twist'
            })
            let message = new ROSLIB.Message({
                linear: { x: this.joystick.vertical, y: 0, z: 0, },
                angular: { x: 0, y: 0, z: this.joystick.horizontal, },
            })
            topic.publish(message)
        },
        disconnect: function() {
            this.ros.close()
        },
        setup3DViewer() {
            this.viewer = new ROS3D.Viewer({
                background: '#cccccc',
                divID: 'div3DViewer',
                width: 150,
                height: 150,
                antialias: true,
                cameraPose: {
                    x: 0.3,
                    y: 0.3,
                    z: 0.5
                },
                fixedFrame: 'odom'
            })

            // Add a grid.
            this.viewer.addObject(new ROS3D.Grid({
                color:'#0181c4',
                cellSize: 0.5,
                num_cells: 20
            }))

            // Setup a client to listen to TFs.
            this.tfClient = new ROSLIB.TFClient({
                ros: this.ros,
                angularThres: 0.01,
                transThres: 0.01,
                rate: 10.0
            })

            // Setup the URDF client.
            this.urdfClient = new ROS3D.UrdfClient({
                ros: this.ros,
                param: 'robot_description',
                tfClient: this.tfClient,
                // We use "path: location.origin + location.pathname"
                // instead of "path: window.location.href" to remove query params,
                // otherwise the assets fail to load
                path: location.origin + location.pathname,
                rootObject: this.viewer.scene,
                loader: ROS3D.STL_LOADER
            })
        },
        unset3DViewer() {
            document.getElementById('div3DViewer').innerHTML = ''
        },
        startDrag() {
            this.dragging = true
            this.x = this.y = 0
        },
        stopDrag() {
            this.dragging = false
            this.x = this.y = 'no'
            this.dragCircleStyle.display = 'none'
            this.resetJoystickVals()
        },
        doDrag(event) {
            if (this.dragging) {
                this.x = event.offsetX
                this.y = event.offsetY
                let ref = document.getElementById('dragstartzone')
                this.dragCircleStyle.display = 'inline-block'

                let minTop = ref.offsetTop - parseInt(this.dragCircleStyle.height) / 2
                let maxTop = minTop + 80
                let top = this.y + minTop
                this.dragCircleStyle.top = `${top}px`

                let minLeft = ref.offsetLeft - parseInt(this.dragCircleStyle.width) / 2
                let maxLeft = minLeft + 80
                let left = this.x + minLeft
                this.dragCircleStyle.left = `${left}px`

                this.setJoystickVals()
            }
        },
        setJoystickVals() {
            this.joystick.vertical = -1 * ((this.y / 80) - 0.5)
            this.joystick.horizontal = -1 * ((this.x / 80) - 0.5)
        },
        resetJoystickVals() {
            this.joystick.vertical = 0
            this.joystick.horizontal = 0
        },

        setCamera: function() {
            let without_wss = this.rosbridge_address.split('wss://')[1]
            console.log(without_wss)
            let domain = without_wss.split('/')[0] + '/' + without_wss.split('/')[1]
            console.log(domain)
            let host = domain + '/cameras'
            let viewer = new MJPEGCANVAS.Viewer({
                divID: 'robotCamera',
                host: host,
                width: 640,
                height: 400,
                topic: '/camera/image_raw',
                ssl: true,})
        },

        sendGoal: function(x, y) {
            let actionClient = new ROSLIB.ActionClient({
                ros : this.ros,
                serverName : '/tortoisebot_as',
                actionName : 'course_web_dev_ros/WaypointActionAction'
            })

            this.action.goal.position.x = x;
            this.action.goal.position.y = y;

            this.goal = new ROSLIB.Goal({
                actionClient : actionClient,
                goalMessage: {
                    ...this.action.goal
                }
            })

            this.goal.on('status', (status) => {
                this.action.status = status
            })

            this.goal.on('feedback', (feedback) => {
                this.action.feedback = feedback
            })

            this.goal.on('result', (result) => {
                this.action.result = result
            })

            this.goal.send()
        },
        cancelGoal: function() {
            this.goal.cancel()
        },
    },
    mounted() {
        // page is ready
        window.addEventListener('mouseup', this.stopDrag)
        this.interval = setInterval(() => {
            if (this.ros != null && this.ros.isConnected) {
                this.ros.getNodes((data) => { }, (error) => { })
            }
        }, 10000)
    },
})