pipeline {
  agent any
  stages {
    stage('Build') {
      steps {
        sh 'printenv'
        dir(path: 'catkin_ws') {
          sh '''
            . /opt/ros/melodic/setup.sh
            catkin build --no-status --verbose
          '''
        }
      }
    }
    stage('Test') {
      steps {
        dir(path: 'catkin_ws') {
          sh '''
            . /opt/ros/melodic/setup.sh
            . devel/setup.sh
            catkin run_tests
            catkin_test_results build --verbose
          '''
        }
      }
    }
  }
  post {
    always {
      archiveArtifacts(artifacts: 'catkin_ws/logs/**/*.log', fingerprint: true)

      script {
        def files = findFiles glob: 'catkin_ws/build/**/test_results/**/*.xml'
        if (files.length > 0) {
          junit 'catkin_ws/build/**/test_results/**/*.xml'
        }
      }
    }
  }
}
