function RenderFrame1(canvasObj, canvasOjb2, videoId) {
  this.videoId = videoId;
  this.imgMain = document.getElementById(videoId);
  
  this.smartCanvas = new HCanvas(canvasObj);
  this.smartCanvas1 = new HCanvas(canvasOjb2);
}

/**
 * 渲染页面
 * @param {*} frame
 */
RenderFrame1.prototype.render = function (frame) {
  if(frame.imageBlob) {
    let obj = this.smartCanvas.getImageWH()
    if (obj.w !== frame.imageWidth || obj.h !== frame.imageHeight) {
      this.smartCanvas.changeImageWH(frame.imageWidth, frame.imageHeight);
      this.smartCanvas1.changeImageWH(frame.imageWidth, frame.imageHeight);
    }
    this.canvasOffset = this.calculateOffset(frame.imageWidth, frame.imageHeight);
    var urlCreator = window.URL || window.webkitURL;
    var imageUrl = urlCreator.createObjectURL(frame.imageBlob);
    requestAnimationFrame(() => {
      this.renderVideo(imageUrl, frame);
      setTimeout(function () {
        urlCreator.revokeObjectURL(imageUrl);
      }, 10);
    });
  }
  if(frame.performance.length > 0) {
    this.renderPerformanceData(frame.performance);
  }
}

/**
 * 性能数据
 * @param {*} performance
 */

RenderFrame1.prototype.renderPerformanceData = function (performance) {
  let performanceHtml = document.querySelector('#performance-message');
  let html = '';
  performance.map((item) =>{
    html += `<li>${item['type_']}: ${item['valueString_']}</li>`
  })
  performanceHtml.innerHTML = html;
}

/**
 * 渲染视频流
 * @param {*} imageUrl
 */

RenderFrame1.prototype.renderVideo = function (imageUrl, frame) {
  var imgMain = this.imgMain
  var _this = this
  imgMain.src = imageUrl;
  imgMain.onload = function () {
    _this.renderFrameStart(frame.smartMsgData, frame.messageShowSelect);
  }
}

RenderFrame1.prototype.renderFrameStart = function (smartMsgData, messageShowSelect) {
  this.smartCanvas.clear();
  this.smartCanvas1.clear();
  if (smartMsgData.length > 0) {
    let parentContainer = document.querySelector('.info-panel-1');
    let parentContainerAlertHtml = document.querySelector('.info-panel-2');
    let htmls = ''
    let htmls2 = ''
    // console.log(smartMsgData)
    smartMsgData.map(item => {
      if (item.boxes.length > 0) {
        let fall = false
        if (typeof item.attributes !== 'undefined') { //  && messageShowSelect.points
          htmls += this.renderAttributes(item.attributes, item.boxes[0])
          if (typeof item.attributes.fall !== 'undefined') {
            fall = true
            htmls2 += this.createAlertHtml(item.attributes.fall, item.boxes[0]);
          }
        }
        this.renderFrameBoxes(item.boxes, fall);
      }
      if (item.subTargets.boxes.length > 0) {
        this.renderFrameBoxes(item.subTargets.boxes);
      }
      if (item.points.length > 0) {
        this.renderFramePoints(item.points)
      }
      if (typeof item.floatMatrixs !== 'undefined') {
        if (item.floatMatrixs === 'segmentation') {
          this.floatMatrixs(item.floatMatrixs)
        } else {
          this.floatMatrixsMask(item.floatMatrixs)
        }
      }
    })
    parentContainer.innerHTML = htmls;
    parentContainerAlertHtml.innerHTML = htmls2;
  }
}

// 渲染轮廓框
RenderFrame1.prototype.renderFrameBoxes = function (boxes, fall) {
  let color = undefined
  if (fall) {
    color = [254, 108, 113];
  }
  boxes.map(item => {
    this.smartCanvas.drawBodyBox(item.p1, item.p2, color);
  })
}

// 渲染骨骼线
RenderFrame1.prototype.renderFramePoints = function (points) {
  points.map(item => {
    this.smartCanvas.drawSkeleton(item.skeletonPoints);
  })
}

// 计算canvas像素和屏幕像素之间的比例关系以及偏移量
RenderFrame1.prototype.calculateOffset = function (width, height) {
  const canvas = document.querySelector('.canvas');
  const parentEle = canvas.parentNode;
  const canvasWidth = canvas.offsetWidth;
  const canvasHeight = canvas.offsetHeight;

  const canvasoffsetX = canvas.offsetLeft + parentEle.offsetLeft;
  const canvasoffsetY = canvas.offsetTop + parentEle.offsetTop;

  const xScale = canvasWidth / width;
  const yScale = canvasHeight / height;
  return {
    xScale,
    yScale,
    offsetX: canvasoffsetX,
    offsetY: canvasoffsetY
  };
}

RenderFrame1.prototype.createTemplateAttributesHtml = function (attributes, className, top, left) {
  let html = `<li class="${className}" style="top:${top}; left:${left}"><ol>`
  if (typeof attributes.type !== 'undefined') {
    html += `<li class="${attributes.typ}">${attributes.type}</li>`
  }
  if (attributes.attributes.length > 0) {
    attributes.attributes.map(val => {
      html += `<li class="${val.type}">${val.type}: ${val.value || ''}</li>`;
    });
  }
  html += '</ol></li>'
  return html;
}

// 渲染属性框
RenderFrame1.prototype.renderAttributes = function (attributes, box) {
  let htmls = '';
  // if (box.p1.y <= 1) {
  //   return;
  // }
  let boxWidth = box.p2.x - box.p1.x;
  let className = '';
  if (boxWidth * this.canvasOffset.xScale > 120) {
    className = 'attribute-panel large';
  } else if (boxWidth * this.canvasOffset.xScale < 80) {
    className = 'attribute-panel small';
  } else {
    className = 'attribute-panel';
  }
  // let x = box.p2.x - box.p1.x
  let y = box.p2.y * this.canvasOffset.yScale - box.p1.y * this.canvasOffset.yScale
  let left = box.p1.x * this.canvasOffset.xScale + 'px';
  let top = box.p1.y * this.canvasOffset.yScale + y + 3 + 'px';
  let html = this.createTemplateAttributesHtml(attributes, className, top, left);
  htmls += html;
  return htmls
}

RenderFrame1.prototype.createTemplateAlertHtml = function (score, top, left) {
  let html = `<li class="alert" style="top:${top}; left:${left}">
    <p class="img"><img src="../assets/images/danger.png" width="16px" alt=""/>有人摔倒啦</p>
  `
  if(typeof score !== 'undefined') {
    html += `<p>score: ${score.toFixed(3)}</p>`;
  }
  html + `</li>`
  return html;
}

// 摔倒弹窗提示
RenderFrame1.prototype.createAlertHtml = function (fall, box) {
  // let parentContainer = document.querySelector('.info-panel-2');
  let html = ''
  if(fall.value === 1) {
    if (box.p1.y <= 1) {
      return;
    }
    let x = box.p2.x * this.canvasOffset.yScale - box.p1.x * this.canvasOffset.yScale
    // let y = (box.p2.y - box.p1.y)
    let left = box.p2.x * this.canvasOffset.xScale - x + 'px';
    let top = box.p1.y * this.canvasOffset.yScale - 3 + 'px';
    html = this.createTemplateAlertHtml(fall.score, top, left);
  }
  return html
}

// 全图分割
RenderFrame1.prototype.floatMatrixs = function (floatMatrixs) {
  if (typeof floatMatrixs.data !== 'undefined') {
    this.smartCanvas1.drawFloatMatrixs(floatMatrixs);
  }
}

// 目标分割
RenderFrame1.prototype.floatMatrixsMask = function (floatMatrixs, box) {
  if (typeof floatMatrixs.data !== 'undefined') {
    this.smartCanvas1.drawFloatMask(floatMatrixs);
  }
  // if (typeof floatMatrixs.data !== 'undefined') {
  //   const maskX = box.p1.x;
  //   const maskY = box.p1.y;
  //   let color = `rgba(${this.colors[0]})`;
  //   const segmentPoints = body.segmentPoints;
  //   this.smartCanvas.drawSegmentBorder(segmentPoints, maskX, maskY, color);
  // }
}
