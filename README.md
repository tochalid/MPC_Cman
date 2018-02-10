## MPC_Cman
<h2><a href="#submission-for-term2-project-5-model-predicted-controller" aria-hidden="true" class="anchor" id="user-content-submission-for-term2-project-5-model-predicted-controller"><svg aria-hidden="true" class="octicon octicon-link" height="16" version="1.1" viewBox="0 0 16 16" width="16"><path fill-rule="evenodd" d="M4 9h1v1H4c-1.5 0-3-1.69-3-3.5S2.55 3 4 3h4c1.45 0 3 1.69 3 3.5 0 1.41-.91 2.72-2 3.25V8.59c.58-.45 1-1.27 1-2.09C10 5.22 8.98 4 8 4H4c-.98 0-2 1.22-2 2.5S3 9 4 9zm9-3h-1v1h1c1 0 2 1.22 2 2.5S13.98 12 13 12H9c-.98 0-2-1.22-2-2.5 0-.83.42-1.64 1-2.09V6.25c-1.09.53-2 1.84-2 3.25C6 11.31 7.55 13 9 13h4c1.45 0 3-1.69 3-3.5S14.5 6 13 6z"></path></svg></a>Submission for Term2 Project 5: Model Predicted Controller</h2>
<p><strong>Objective:</strong> Implement a Model Predictive Control (MPC) algorithm in C++ to control steering and throttle of a simulated vehicle driving a simulator track of reference waypoints.</p>
<p>Passing the project requires a successful simulator track run and documentation, pls see project <a href="https://review.udacity.com/#!/rubrics/896/view" rel="nofollow">rubics</a>. Boilerplate code provided by Udacity under <a href="https://github.com/udacity/CarND-MPC-Project">https://github.com/udacity/CarND-MPC-Project</a>.</p>
<blockquote>
<p>Improvements made to:</p>
</blockquote>
<ul>
<li>src/<a href="/tochalid/MPC_Cman/blob/master/src/main.cpp">main.cpp</a></li>
<li>src/<a href="/tochalid/MPC_Cman/blob/master/src/MPC.cpp">MPC.cpp</a></li>
</ul>
<h2><a href="#implementation" aria-hidden="true" class="anchor" id="user-content-implementation"><svg aria-hidden="true" class="octicon octicon-link" height="16" version="1.1" viewBox="0 0 16 16" width="16"><path fill-rule="evenodd" d="M4 9h1v1H4c-1.5 0-3-1.69-3-3.5S2.55 3 4 3h4c1.45 0 3 1.69 3 3.5 0 1.41-.91 2.72-2 3.25V8.59c.58-.45 1-1.27 1-2.09C10 5.22 8.98 4 8 4H4c-.98 0-2 1.22-2 2.5S3 9 4 9zm9-3h-1v1h1c1 0 2 1.22 2 2.5S13.98 12 13 12H9c-.98 0-2-1.22-2-2.5 0-.83.42-1.64 1-2.09V6.25c-1.09.53-2 1.84-2 3.25C6 11.31 7.55 13 9 13h4c1.45 0 3-1.69 3-3.5S14.5 6 13 6z"></path></svg></a>Implementation</h2>
<h3><a href="#the-model" aria-hidden="true" class="anchor" id="user-content-the-model"><svg aria-hidden="true" class="octicon octicon-link" height="16" version="1.1" viewBox="0 0 16 16" width="16"><path fill-rule="evenodd" d="M4 9h1v1H4c-1.5 0-3-1.69-3-3.5S2.55 3 4 3h4c1.45 0 3 1.69 3 3.5 0 1.41-.91 2.72-2 3.25V8.59c.58-.45 1-1.27 1-2.09C10 5.22 8.98 4 8 4H4c-.98 0-2 1.22-2 2.5S3 9 4 9zm9-3h-1v1h1c1 0 2 1.22 2 2.5S13.98 12 13 12H9c-.98 0-2-1.22-2-2.5 0-.83.42-1.64 1-2.09V6.25c-1.09.53-2 1.84-2 3.25C6 11.31 7.55 13 9 13h4c1.45 0 3-1.69 3-3.5S14.5 6 13 6z"></path></svg></a>The Model</h3>
<blockquote>
<p>Detailed description of the used model including state, actuators and update equations.</p>
</blockquote>
<p>The MPC uses a global kinematic model (bicycle) to resolve the vehicle state for a prediction horizon. The state is described by 6 dimensions: <strong>x</strong>- and <strong>y</strong>-coordinate of vehicle position, vehicle heading <strong>psi</strong>, vehicle velocity <strong>v</strong>, cross track error <strong>cte</strong> as deviation from reference line, and the error in heading <strong>epsi</strong>. The horizon T is <strong>N</strong> steps of duration <strong>dt</strong>. The IPOPT solver uses the initial state and actuators (steering <strong>delta</strong> and throttle <strong>a</strong> within specific constraints) returning a vector of control inputs for each time step  <strong>[delta_1..N a_1..N]</strong> that minimize the cost function of the model. Actuations for the next time step are pushed to the vehicle state and the prediction loop is repeated. The model is simplified eg. ignoring tire dynamics.</p>
<p><a href="/tochalid/MPC_Cman/blob/master/mpc.png" target="_blank"><img src="/tochalid/MPC_Cman/raw/master/mpc.png" alt="Image" style="max-width:100%;"></a></p>
<h3><a href="#timestep-length-and-elapsed-duration-n--dt" aria-hidden="true" class="anchor" id="user-content-timestep-length-and-elapsed-duration-n--dt"><svg aria-hidden="true" class="octicon octicon-link" height="16" version="1.1" viewBox="0 0 16 16" width="16"><path fill-rule="evenodd" d="M4 9h1v1H4c-1.5 0-3-1.69-3-3.5S2.55 3 4 3h4c1.45 0 3 1.69 3 3.5 0 1.41-.91 2.72-2 3.25V8.59c.58-.45 1-1.27 1-2.09C10 5.22 8.98 4 8 4H4c-.98 0-2 1.22-2 2.5S3 9 4 9zm9-3h-1v1h1c1 0 2 1.22 2 2.5S13.98 12 13 12H9c-.98 0-2-1.22-2-2.5 0-.83.42-1.64 1-2.09V6.25c-1.09.53-2 1.84-2 3.25C6 11.31 7.55 13 9 13h4c1.45 0 3-1.69 3-3.5S14.5 6 13 6z"></path></svg></a>Timestep Length and Elapsed Duration (N &amp; dt)</h3>
<blockquote>
<p>Discussion of the reasoning behind the chosen N (timestep length) and dt (elapsed duration between timesteps) values including previous values tried.</p>
</blockquote>
<p>N determines the number of steps for which the cost function will be optimized for by the solver. Large N lead to consuming solver calculations impacting MPC performance. dt determines the distance between consequent actuations and large dt result in larger devitations from the reference trajectory. Increasing oscillations indicate thresholds for the specific configuration. Choise is <strong>N=8</strong> and <strong>dt=0.12s</strong> where the vehicle is able to properly navigate the track and resulting in prediction horizon <strong>T=~1sec</strong> with fixed set of weights and including latency. Thus, models optimizer is looking at a 1sec duration to determine a corrective trajectory. Adjusting N or dt mainly causes erratic behavior, oszillation, soon off track running. Other values tried include 20/0.05, 10/0.10, 6/0.15, 5/0.2, and more.</p>
<h3><a href="#polynomial-fitting-and-mpc-preprocessing" aria-hidden="true" class="anchor" id="user-content-polynomial-fitting-and-mpc-preprocessing"><svg aria-hidden="true" class="octicon octicon-link" height="16" version="1.1" viewBox="0 0 16 16" width="16"><path fill-rule="evenodd" d="M4 9h1v1H4c-1.5 0-3-1.69-3-3.5S2.55 3 4 3h4c1.45 0 3 1.69 3 3.5 0 1.41-.91 2.72-2 3.25V8.59c.58-.45 1-1.27 1-2.09C10 5.22 8.98 4 8 4H4c-.98 0-2 1.22-2 2.5S3 9 4 9zm9-3h-1v1h1c1 0 2 1.22 2 2.5S13.98 12 13 12H9c-.98 0-2-1.22-2-2.5 0-.83.42-1.64 1-2.09V6.25c-1.09.53-2 1.84-2 3.25C6 11.31 7.55 13 9 13h4c1.45 0 3-1.69 3-3.5S14.5 6 13 6z"></path></svg></a>Polynomial Fitting and MPC Preprocessing</h3>
<blockquote>
<p>Description of polynomial fitting and preprocessing of waypoints, vehicle state and actuators prior to the MPC procedure.</p>
</blockquote>
<p>The reference trajectory is calculated with fitting a <strong>3rd-order polynomial</strong> to waypoints returned by the simulator. In advance to the fitting the waypoints are converted to car-coordinate scheme. Waypoints are shifted and rotated. Then cte and epsi are calculated by solving the polynomial at <strong>x=0</strong>. Thus, the angle of the reference trajectory rests with the x-axis at the origin.</p>
<p>Weights were tuned so that large actuations were penalized but change rates of <strong>delta</strong> and <strong>a</strong> still set reactive to curvey road conditions. This helps for smoother steering but a brief tendency to overshoot running high speed. Larger weight on <strong>cte</strong> caused instability. Orientation error <strong>psi</strong> was penalized stronger to allow curvey road adaption. Weight of the velocity difference <strong>dif v</strong> gradually increased until <strong>reference velocity of ~90mph</strong> achieved. Pls see final weight values:</p>
<ul>
<li>weight cte: 20</li>
<li>weight epsi: 3000</li>
<li>weight dif v: 1000</li>
<li>weight delta: 5000</li>
<li>weight a: 500</li>
<li>weight delra rate: 500</li>
<li>weight a rate: 50</li>
</ul>
<h3><a href="#model-predictive-control-with-latency" aria-hidden="true" class="anchor" id="user-content-model-predictive-control-with-latency"><svg aria-hidden="true" class="octicon octicon-link" height="16" version="1.1" viewBox="0 0 16 16" width="16"><path fill-rule="evenodd" d="M4 9h1v1H4c-1.5 0-3-1.69-3-3.5S2.55 3 4 3h4c1.45 0 3 1.69 3 3.5 0 1.41-.91 2.72-2 3.25V8.59c.58-.45 1-1.27 1-2.09C10 5.22 8.98 4 8 4H4c-.98 0-2 1.22-2 2.5S3 9 4 9zm9-3h-1v1h1c1 0 2 1.22 2 2.5S13.98 12 13 12H9c-.98 0-2-1.22-2-2.5 0-.83.42-1.64 1-2.09V6.25c-1.09.53-2 1.84-2 3.25C6 11.31 7.55 13 9 13h4c1.45 0 3-1.69 3-3.5S14.5 6 13 6z"></path></svg></a>Model Predictive Control with Latency</h3>
<blockquote>
<p>Details on the 100 millisecond latency handling.</p>
</blockquote>
<p>Latency of 100ms is modeled in projecting the x-position of the current state and speed into the future. The <strong>x-position with 0.1sec updated</strong> and all unchanged state variables are passed to the solver.</p>
<p>Note: Steering angle is flipped (multiplied by -1) to reflect turning conventions of simulator and MPC model. The calculated steer value is normalized and flipped before back-passing to the simulator.</p>
<h2><a href="#compilation-and-simulation" aria-hidden="true" class="anchor" id="user-content-compilation-and-simulation"><svg aria-hidden="true" class="octicon octicon-link" height="16" version="1.1" viewBox="0 0 16 16" width="16"><path fill-rule="evenodd" d="M4 9h1v1H4c-1.5 0-3-1.69-3-3.5S2.55 3 4 3h4c1.45 0 3 1.69 3 3.5 0 1.41-.91 2.72-2 3.25V8.59c.58-.45 1-1.27 1-2.09C10 5.22 8.98 4 8 4H4c-.98 0-2 1.22-2 2.5S3 9 4 9zm9-3h-1v1h1c1 0 2 1.22 2 2.5S13.98 12 13 12H9c-.98 0-2-1.22-2-2.5 0-.83.42-1.64 1-2.09V6.25c-1.09.53-2 1.84-2 3.25C6 11.31 7.55 13 9 13h4c1.45 0 3-1.69 3-3.5S14.5 6 13 6z"></path></svg></a>Compilation and Simulation</h2>
<blockquote>
<p><strong>build instructions</strong>: run shell-commands from the project directory</p>
</blockquote>
<ol>
<li>Clone this repo.</li>
<li>Make a build directory: <code>mkdir build &amp;&amp; cd build</code></li>
<li>Compile: <code>cmake .. &amp;&amp; make</code></li>
<li>Run it: <code>./mpc</code>.</li>
<li>Run and start the MPC simulator (resolution: 600x800, Mode: fantastic)</li>
</ol>
<h2><a href="#environment" aria-hidden="true" class="anchor" id="user-content-environment"><svg aria-hidden="true" class="octicon octicon-link" height="16" version="1.1" viewBox="0 0 16 16" width="16"><path fill-rule="evenodd" d="M4 9h1v1H4c-1.5 0-3-1.69-3-3.5S2.55 3 4 3h4c1.45 0 3 1.69 3 3.5 0 1.41-.91 2.72-2 3.25V8.59c.58-.45 1-1.27 1-2.09C10 5.22 8.98 4 8 4H4c-.98 0-2 1.22-2 2.5S3 9 4 9zm9-3h-1v1h1c1 0 2 1.22 2 2.5S13.98 12 13 12H9c-.98 0-2-1.22-2-2.5 0-.83.42-1.64 1-2.09V6.25c-1.09.53-2 1.84-2 3.25C6 11.31 7.55 13 9 13h4c1.45 0 3-1.69 3-3.5S14.5 6 13 6z"></path></svg></a>Environment</h2>
<ul>
<li><strong>OS Setup:</strong> Ubuntu 16.4, for details pls see <a href="https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d" rel="nofollow">here</a></li>
<li><strong>Ipopt and CppAD packages:</strong> Please refer to <a href="https://github.com/udacity/CarND-MPC-Project/blob/master/install_Ipopt_CppAD.md">this document</a> for installation instructions.</li>
<li><strong>Eigen package:</strong> <a href="http://eigen.tuxfamily.org/index.php?title=Main_Page" rel="nofollow">Eigen</a> is already part of the repo.</li>
<li><strong>Simulator:</strong> The project involves Term 2 Simulator which can be downloaded here: <a href="https://github.com/udacity/self-driving-car-sim/releases">https://github.com/udacity/self-driving-car-sim/releases</a>. A server package uWebSocketIO is setting up a connection from the C++ program to the simulator, which acts as the host. Read the <a href="/tochalid/MPC_Cman/blob/master/DATA.md">DATA.md</a> for a description of the data sent back from the simulator.</li>
</ul>
</article>
  </div>

  </div>

  <button type="button" data-facebox="#jump-to-line" data-facebox-class="linejump" data-hotkey="l" class="d-none">Jump to Line</button>
  <div id="jump-to-line" style="display:none">
    <!-- '"` --><!-- </textarea></xmp> --></option></form><form accept-charset="UTF-8" action="" class="js-jump-to-line-form" method="get"><div style="margin:0;padding:0;display:inline"><input name="utf8" type="hidden" value="&#x2713;" /></div>
      <input class="form-control linejump-input js-jump-to-line-field" type="text" placeholder="Jump to line&hellip;" aria-label="Jump to line" autofocus>
      <button type="submit" class="btn">Go</button>
</form>  </div>


  </div>
  <div class="modal-backdrop js-touch-events"></div>
</div>

    </div>
  </div>

  </div>

      
<div class="footer container-lg px-3" role="contentinfo">
  <div class="position-relative d-flex flex-justify-between py-6 mt-6 f6 text-gray border-top border-gray-light ">
    <ul class="list-style-none d-flex flex-wrap ">
      <li class="mr-3">&copy; 2018 <span title="0.17497s from unicorn-1707316450-bkmtp">GitHub</span>, Inc.</li>
        <li class="mr-3"><a href="https://help.github.com/articles/github-terms-of-service/" data-ga-click="Footer, go to terms, text:terms">Terms</a></li>
        <li class="mr-3"><a href="https://github.com/site/privacy" data-ga-click="Footer, go to privacy, text:privacy">Privacy</a></li>
        <li class="mr-3"><a href="https://help.github.com/articles/github-security/" data-ga-click="Footer, go to security, text:security">Security</a></li>
        <li class="mr-3"><a href="https://status.github.com/" data-ga-click="Footer, go to status, text:status">Status</a></li>
        <li><a href="https://help.github.com" data-ga-click="Footer, go to help, text:help">Help</a></li>
    </ul>

    <a href="https://github.com" aria-label="Homepage" class="footer-octicon" title="GitHub">
      <svg aria-hidden="true" class="octicon octicon-mark-github" height="24" version="1.1" viewBox="0 0 16 16" width="24"><path fill-rule="evenodd" d="M8 0C3.58 0 0 3.58 0 8c0 3.54 2.29 6.53 5.47 7.59.4.07.55-.17.55-.38 0-.19-.01-.82-.01-1.49-2.01.37-2.53-.49-2.69-.94-.09-.23-.48-.94-.82-1.13-.28-.15-.68-.52-.01-.53.63-.01 1.08.58 1.23.82.72 1.21 1.87.87 2.33.66.07-.52.28-.87.51-1.07-1.78-.2-3.64-.89-3.64-3.95 0-.87.31-1.59.82-2.15-.08-.2-.36-1.02.08-2.12 0 0 .67-.21 2.2.82.64-.18 1.32-.27 2-.27.68 0 1.36.09 2 .27 1.53-1.04 2.2-.82 2.2-.82.44 1.1.16 1.92.08 2.12.51.56.82 1.27.82 2.15 0 3.07-1.87 3.75-3.65 3.95.29.25.54.73.54 1.48 0 1.07-.01 1.93-.01 2.2 0 .21.15.46.55.38A8.013 8.013 0 0 0 16 8c0-4.42-3.58-8-8-8z"/></svg>
</a>
    <ul class="list-style-none d-flex flex-wrap ">
        <li class="mr-3"><a href="https://github.com/contact" data-ga-click="Footer, go to contact, text:contact">Contact GitHub</a></li>
      <li class="mr-3"><a href="https://developer.github.com" data-ga-click="Footer, go to api, text:api">API</a></li>
      <li class="mr-3"><a href="https://training.github.com" data-ga-click="Footer, go to training, text:training">Training</a></li>
      <li class="mr-3"><a href="https://shop.github.com" data-ga-click="Footer, go to shop, text:shop">Shop</a></li>
        <li class="mr-3"><a href="https://github.com/blog" data-ga-click="Footer, go to blog, text:blog">Blog</a></li>
        <li><a href="https://github.com/about" data-ga-click="Footer, go to about, text:about">About</a></li>

    </ul>
  </div>
</div>



  <div id="ajax-error-message" class="ajax-error-message flash flash-error">
    <svg aria-hidden="true" class="octicon octicon-alert" height="16" version="1.1" viewBox="0 0 16 16" width="16"><path fill-rule="evenodd" d="M8.865 1.52c-.18-.31-.51-.5-.87-.5s-.69.19-.87.5L.275 13.5c-.18.31-.18.69 0 1 .19.31.52.5.87.5h13.7c.36 0 .69-.19.86-.5.17-.31.18-.69.01-1L8.865 1.52zM8.995 13h-2v-2h2v2zm0-3h-2V6h2v4z"/></svg>
    <button type="button" class="flash-close js-ajax-error-dismiss" aria-label="Dismiss error">
      <svg aria-hidden="true" class="octicon octicon-x" height="16" version="1.1" viewBox="0 0 12 16" width="12"><path fill-rule="evenodd" d="M7.48 8l3.75 3.75-1.48 1.48L6 9.48l-3.75 3.75-1.48-1.48L4.52 8 .77 4.25l1.48-1.48L6 6.52l3.75-3.75 1.48 1.48z"/></svg>
    </button>
    You can't perform that action at this time.
  </div>


    
    <script crossorigin="anonymous" integrity="sha512-MO2ZtIEZrtDlHQRpl/db39VEpjw/+MfQjkgZhl52kxNrXqYwEKwe0oEa/IRuvPRN5v9UUrznRAoA2AgP8s/OIw==" src="https://assets-cdn.github.com/assets/frameworks-30ed99b48119.js" type="application/javascript"></script>
    
    <script async="async" crossorigin="anonymous" integrity="sha512-MJ1cJA/+TxAPo6uxf/bsi406QgJY9mb1k/MIDgOMzJTVxm9Vc0kyeWkdT9wq+DVa1Y2jb/zH7r0cq48+7zP15w==" src="https://assets-cdn.github.com/assets/github-309d5c240ffe.js" type="application/javascript"></script>
    
    
    
    
  <div class="js-stale-session-flash stale-session-flash flash flash-warn flash-banner d-none">
    <svg aria-hidden="true" class="octicon octicon-alert" height="16" version="1.1" viewBox="0 0 16 16" width="16"><path fill-rule="evenodd" d="M8.865 1.52c-.18-.31-.51-.5-.87-.5s-.69.19-.87.5L.275 13.5c-.18.31-.18.69 0 1 .19.31.52.5.87.5h13.7c.36 0 .69-.19.86-.5.17-.31.18-.69.01-1L8.865 1.52zM8.995 13h-2v-2h2v2zm0-3h-2V6h2v4z"/></svg>
    <span class="signed-in-tab-flash">You signed in with another tab or window. <a href="">Reload</a> to refresh your session.</span>
    <span class="signed-out-tab-flash">You signed out in another tab or window. <a href="">Reload</a> to refresh your session.</span>
  </div>
  <div class="facebox" id="facebox" style="display:none;">
  <div class="facebox-popup">
    <div class="facebox-content" role="dialog" aria-labelledby="facebox-header" aria-describedby="facebox-description">
    </div>
    <button type="button" class="facebox-close js-facebox-close" aria-label="Close modal">
      <svg aria-hidden="true" class="octicon octicon-x" height="16" version="1.1" viewBox="0 0 12 16" width="12"><path fill-rule="evenodd" d="M7.48 8l3.75 3.75-1.48 1.48L6 9.48l-3.75 3.75-1.48-1.48L4.52 8 .77 4.25l1.48-1.48L6 6.52l3.75-3.75 1.48 1.48z"/></svg>
    </button>
  </div>
</div>

  

  </body>
</html>

