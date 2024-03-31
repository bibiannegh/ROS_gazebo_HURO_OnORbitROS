
document.addEventListener('DOMContentLoaded', function() {
    var codeBlocks = document.querySelectorAll('pre');
    codeBlocks.forEach(function(codeBlock) {
        var copyButton = document.createElement('button');
        copyButton.className = 'copy-button';
        copyButton.innerHTML = '<i class="fa fa-copy"></i>';
        copyButton.addEventListener('click', function() {
            var codeText = codeBlock.querySelector('code').innerText;
            navigator.clipboard.writeText(codeText).then(function() {
                console.log('Code copied to clipboard');
            }, function(err) {
                console.error('Failed to copy code: ', err);
            });
        });
        codeBlock.parentNode.insertBefore(copyButton, codeBlock);
        positionCopyButton(copyButton, codeBlock); // Call the function to position the copy button
    });
});

// Function to position the copy button
function positionCopyButton(copyButton, codeBlock) {
    // Set the copy button's position to the top right corner of the code block
    var codeBlockRect = codeBlock.getBoundingClientRect();
    copyButton.style.position = 'absolute';
    copyButton.style.top = (codeBlockRect.top-30) + 'px';
    copyButton.style.right = (codeBlockRect.left+105) + 'px';
}