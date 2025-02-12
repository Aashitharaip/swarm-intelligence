
function canvas_resize(canvas,aspectRatio){
    var hasChanged=false;
    var tab=document.getElementById("mainTable");
    var yPixCanvasTop=tab.getBoundingClientRect().bottom
    yPixCanvasTop=Math.min(yPixCanvasTop,0.3*window.innerHeight);
    var newWidth=Math.round(document.body.offsetWidth); 
    var newHeight=Math.round(window.innerHeight-yPixCanvasTop); 
    if (canvas.width!=newWidth){
        hasChanged=true;
        canvas.width  = newWidth;
    }
    if (canvas.height != newHeight){
	    hasChanged=true;
        canvas.height  = newHeight;
    }

    if(hasChanged){
        center_x=0.50*canvas.width; 
        center_y=0.48*canvas.height;
        var refDim=Math.min(canvas.width,canvas.height*aspectRatio);
        scale=refDim/sizePhys;  
    }
    return hasChanged;
}

