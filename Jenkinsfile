node (label: 'aws-jenkins-slave-worker') {
  stage('Checking out code') {
    checkout scm: [
      $class: 'GitSCM',
      branches: scm.branches,
      doGenerateSubmoduleConfigurations: false,
      extensions: scm.extensions +
        [[$class: 'SubmoduleOption', parentCredentials: true, timeout: 300]] +
        [$class: 'GitLFSPull'],
      userRemoteConfigs: scm.userRemoteConfigs
    ]
  }

  def shortCommit = sh(returnStdout: true, script: "git log -1 --format='%h' --abbrev=7").trim()

  stage('Running pylint') {
    docker.image('python:2').inside('-u root') {
      sh "pip install --no-cache-dir pylint"
      sh "./pylint"
    }
  }

  stage('Running unittests') {
    docker.image('python:2').inside('-u root') {
      sh "pip install -r requirements.txt"
      sh "python -m unittest discover"
    }
  }
}
